"""只读STEP，记录名义几何尺寸并生成三视图；不连接机械臂。"""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / ".runtime_deps/cad_step"))
import OCP
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Pln, gp_Pnt, gp_Dir, gp_Lin
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector

SOURCE = ROOT / "reference/models/battery_prop_20261011/电池道具.stp"
OUTPUT = ROOT / "artifacts/model_battery_20261011"


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    low, high = box.CornerMin(), box.CornerMax()
    return [low.X(), low.Y(), low.Z(), high.X(), high.Y(), high.Z()]


def dimensions(shape):
    b = bounds(shape)
    return [round(b[i + 3] - b[i], 5) for i in range(3)]


def count(shape, kind):
    ex, total = TopExp_Explorer(shape, kind), 0
    while ex.More():
        total += 1
        ex.Next()
    return total


def section_at(shape, z):
    op = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)), False)
    op.Build()
    if not op.IsDone():
        raise RuntimeError("截面计算失败")
    return op.Shape()


def preview(shape):
    BRepMesh_IncrementalMesh(shape, 0.15, False, 0.35, True)
    tris = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        loc = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(TopoDS.Face(ex.Current()), loc)
        for i in range(1, mesh.NbTriangles() + 1):
            tri = []
            for j in mesh.Triangle(i).Get():
                p = mesh.Node(j).Transformed(loc.Transformation())
                tri.append((p.X(), -p.Y(), 50 - p.Z()))
            tris.append(np.array(tri))
        ex.Next()
    im = Image.new("RGB", (1500, 730), "#f5f7fa")
    draw = ImageDraw.Draw(im)
    fontpath = Path("C:/Windows/Fonts/msyh.ttc")
    font = ImageFont.truetype(str(fontpath), 19)
    title = ImageFont.truetype(str(fontpath), 29)
    draw.text((34, 20), "电池道具 STEP｜名义尺寸（毫米）", fill="#18334c", font=title)
    draw.text((34, 65), "仅显示模型几何，不代表实物公差或机械臂坐标标定", fill="#526579", font=font)
    panels = [(30, np.array([1.4, -2., 1.4]), "立体视图"),
              (525, np.array([0., -1., 0.]), "正视：X / 向上"),
              (1020, np.array([1., 0., 0.]), "侧视：Y / 向上")]
    for left, view, label in panels:
        view = view / np.linalg.norm(view)
        right = np.cross(view, [0., 0., 1.]); right /= np.linalg.norm(right)
        up = np.cross(right, view)
        center = np.array([0., 0., 40.])
        draw.rounded_rectangle((left, 110, left + 450, 620), 12, fill="white", outline="#d9e0e9")
        draw.text((left + 20, 130), label, fill="#18334c", font=font)
        for tri in sorted(tris, key=lambda t: float(t.mean(axis=0) @ view)):
            xy = [((p-center) @ right, (p-center) @ up) for p in tri]
            scale = 3.4 if left == 30 else 4.3
            pts = [(left + 225 + a*scale, 345-b*scale) for a, b in xy]
            n = np.cross(tri[1]-tri[0], tri[2]-tri[0]); length = np.linalg.norm(n)
            if length == 0:
                continue
            light = 0.68 + 0.32*abs(float(n/length @ np.array([0.3, -0.4, 0.866])))
            color = tuple(int(v*light) for v in (209, 77, 69))
            draw.polygon(pts, fill=color)
        notes = {30: "外形：70 × 80 × 80", 525: "篮体高50；提手凸出30", 1020: "横梁截面：26 × 10"}
        draw.text((left + 24, 560), notes[left], fill="#18334c", font=font)
    draw.text((34, 650), "横梁中心高约75；开口中部净距50、中心净高20，圆角处净空较小。", fill="#34495e", font=font)
    draw.text((34, 684), "摆正变换：X'=X，Y'=-Y，Z'=50-Z。现场邻篮间距约10毫米，为用户估计值。", fill="#526579", font=font)
    im.save(OUTPUT / "battery_model_views.png")
    return len(tris)


def main():
    data = SOURCE.read_bytes()
    if b"SI_UNIT(.MILLI.,.METRE.)" not in data:
        raise ValueError("本模型单位未确认为毫米")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(SOURCE)) != IFSelect_RetDone:
        raise ValueError("STEP读取失败")
    reader.TransferRoots()
    shape = reader.OneShape()
    valid = BRepCheck_Analyzer(shape).IsValid()
    if not valid or count(shape, TopAbs_SOLID) != 1:
        raise ValueError("实体无效或数量与预期不符")
    body_section, bar_section = section_at(shape, 25), section_at(shape, -25)
    ex = TopExp_Explorer(section_at(shape, -10), TopAbs_EDGE)
    negative, positive = [], []
    while ex.More():
        b = bounds(ex.Current())
        if b[3] < 0: negative.append(b[3])
        if b[0] > 0: positive.append(b[0])
        ex.Next()
    inter = IntCurvesFace_ShapeIntersector()
    inter.Load(shape, 1e-7)
    inter.Perform(gp_Lin(gp_Pnt(0, 0, -100), gp_Dir(0, 0, 1)), 0, 200)
    z = sorted(set(round(inter.Pnt(i).Z(), 5) for i in range(1, inter.NbPnt()+1)))
    assert z == [-30., -20., 0., 50.], z
    overall = dimensions(shape)
    assert np.allclose(overall, [70, 80, 80], atol=1e-5)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    result = dict(source=str(SOURCE.relative_to(ROOT)), source_sha256=hashlib.sha256(data).hexdigest(),
                  units="mm", engine="OpenCascade/OCP " + OCP.__version__, valid_solid=valid,
                  solid_count=count(shape, TopAbs_SOLID), face_count=count(shape, TopAbs_FACE),
                  cad_bounds_mm=[round(v, 7) for v in bounds(shape)], overall_xyz_mm=overall,
                  body_midsection_xy_mm=dimensions(body_section)[:2], body_center_height_mm=z[3]-z[2],
                  handle_protrusion_height_mm=z[2]-z[0], crossbar_midsection_xy_mm=dimensions(bar_section)[:2],
                  crossbar_center_thickness_mm=z[1]-z[0], opening_width_at_cad_z_minus10_mm=round(min(positive)-max(negative), 5),
                  centerline_surface_z_mm=z, opening_center_height_mm=z[2]-z[1],
                  crossbar_center_height_above_base_mm=z[3]-(z[0]+z[1])/2,
                  upright_model_transform="X'=X; Y'=-Y; Z'=50-Z (display only; not robot calibration)",
                  interpretation="Dimensions are nominal CAD geometry; rounded opening is not a full clear rectangle; no physical tolerance or mass measured.",
                  site_report=dict(source="user 2026-10-11", total_baskets=4, neighboring_obstacle_baskets=3,
                                   approximate_gap_mm=10, gap_measured=False, placement_uncertainty_mm=None,
                                   layout_and_minimum_gap_surveyed=False))
    result["preview_triangles"] = preview(shape)
    (OUTPUT / "measurements.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({k:result[k] for k in ["overall_xyz_mm","body_midsection_xy_mm","body_center_height_mm","handle_protrusion_height_mm","crossbar_midsection_xy_mm","crossbar_center_thickness_mm","opening_width_at_cad_z_minus10_mm","opening_center_height_mm","valid_solid"]},ensure_ascii=False))


if __name__ == "__main__":
    main()
