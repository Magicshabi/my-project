"""Independent collection format and offline checks; never opens serial ports."""
import hashlib
import json
import math
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LIMITS_FILE = ROOT.parent / '八点推进_20261008/data/joint_limits_20261010.json'
COLORS = ('红', '黄', '蓝', '绿')
TARGETS = ('P1', 'P2', 'P3')
STAGES = ('source_above_open', 'source_down_open', 'source_grip', 'source_clear',
          'destination_above', 'destination_down', 'destination_release', 'destination_clear')
SCHEMA = 'nexarm.continuous-collection/1'
D6A = 'd6a_import'
GRIP_SQUEEZE = 200  # d6a导入：S3夹紧目标=示教ID6+200（封顶软件上限），执行器不再叠加


def now():
    return datetime.now(timezone(timedelta(hours=8))).isoformat()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def limits():
    doc = read(LIMITS_FILE)
    require(isinstance(doc, dict), '软件限位文件必须是对象')
    require(doc.get('schema') == 'nexarm.host-joint-limits/1'
            and doc.get('servo_ids') == list(range(1, 7)), '软件限位格式错误')
    bounds = doc.get('bounds')
    require(isinstance(bounds, list) and len(bounds) == 6, '软件限位需要六轴')
    for pair in bounds:
        require(isinstance(pair, list) and len(pair) == 2
                and all(type(v) is int for v in pair) and 0 <= pair[0] < pair[1] <= 4095,
                '软件限位范围错误')
    return bounds


def counts(values):
    require(isinstance(values, list) and len(values) == 6
            and all(type(v) is int for v in values), '必须有ID1..6六个整数刻度')
    require(all(lo <= v <= hi for v, (lo, hi) in zip(values, limits())), '刻度超出软件范围')
    return values


def route_path(root, color=None, target=None):
    if color is None and target is None:
        return Path(root) / '初始位采集/home.json'
    require(color in COLORS and target in TARGETS, '颜色或目标区域无效')
    return Path(root) / f'{color}篮采集/{color}到{target}/route.json'


def paths(root):
    yield route_path(root), None, None
    for color in COLORS:
        for target in TARGETS:
            yield route_path(root, color, target), color, target


def identity(color=None, target=None):
    return dict(kind='home' if color is None else 'route', color=color, destination=target)


def load_scene(root):
    scene = read(Path(root) / 'scene.json')
    require(isinstance(scene, dict), '场地文件必须是对象')
    require(scene.get('schema') == 'nexarm.continuous-scene/1'
            and isinstance(scene.get('layout_id'), str) and scene['layout_id'].strip(), '场地标识缺失')
    require(scene.get('source_colors') == list(COLORS) and scene.get('destinations') == list(TARGETS),
            '场地颜色/目标配置不匹配')
    return scene


def template(root, color=None, target=None):
    scene = load_scene(root)
    return dict(schema=SCHEMA, **identity(color, target), layout_id=scene['layout_id'],
                scene_sha256=sha(Path(root) / 'scene.json'), units='raw_encoder_counts',
                servo_ids=list(range(1, 7)), limits_sha256=sha(LIMITS_FILE),
                configuration=None, points=[], complete=False, execution_verified=False)


def validate(doc, root, color=None, target=None, complete=False):
    require(isinstance(doc, dict) and doc.get('schema') == SCHEMA, '主文件schema不匹配')
    require(all(doc.get(k) == v for k, v in identity(color, target).items()), '路线身份与目录不符')
    scene = load_scene(root)
    require(doc.get('layout_id') == scene['layout_id']
            and doc.get('scene_sha256') == sha(Path(root) / 'scene.json'), '场地版本改变')
    require(doc.get('limits_sha256') == sha(LIMITS_FILE), '软件限位绑定改变')
    require(doc.get('units') == 'raw_encoder_counts' and doc.get('servo_ids') == list(range(1, 7)),
            '单位或舵机顺序不匹配')
    rows = doc.get('points')
    expected = 1 if color is None else 8
    require(isinstance(rows, list) and len(rows) <= expected, '点数异常')
    require(doc.get('complete') is (len(rows) == expected), 'complete与实际点数不一致')
    require(doc.get('execution_verified') is False, '采集主文件不得自行标记运动验收通过')
    if complete:
        require(len(rows) == expected, f'需要{expected}个点，实际{len(rows)}')
    sources = {r.get('source', 'serial_capture') for r in rows if isinstance(r, dict)}
    require(len(sources) <= 1, '同一路线不得混用串口采集与d6a导入')
    if sources == {D6A}:
        require(doc.get('configuration') is None, 'd6a导入不得伪造设备配置')
    elif rows:
        cfg = doc.get('configuration')
        require(isinstance(cfg, dict) and bool(cfg), '缺少设备配置')
        require(isinstance(cfg.get('version'), dict)
                and (cfg['version'].get('esp_str'), cfg['version'].get('at32_str')) == ('1.0.2', '1.0.42'),
                '固件版本未核准')
        for key, size in (('limits', 12), ('kinematics', 9)):
            require(isinstance(cfg.get(key), list) and len(cfg[key]) == size
                    and all(type(v) in (int, float) and math.isfinite(v) for v in cfg[key]),
                    '设备配置缺少完整' + key)
        offsets = cfg.get('offsets')
        require(isinstance(offsets, dict) and set(offsets) == {str(i) for i in range(1, 7)}
                and all(type(v) is int for v in offsets.values()), '设备配置缺少六轴零偏')
    close_target = None
    for i, row in enumerate(rows, 1):
        require(isinstance(row, dict) and type(row.get('point')) is int and row['point'] == i,
                '点号必须唯一且按顺序新增')
        require(row.get('stage') == ('home' if color is None else STAGES[i - 1]), '阶段名称不匹配')
        actual = counts(row.get('encoder_positions'))
        goal = row.get('gripper_target_counts')
        require(type(goal) is int and limits()[5][0] <= goal <= limits()[5][1], '缺少有效ID6目标刻度')
        require(row.get('physical_pose_confirmed') is True
                and row.get('motor_driven_pose_confirmed_by_operator') is True
                and isinstance(row.get('observation'), str) and row['observation'].strip(), '缺少摆位依据')
        require(isinstance(row.get('captured_at'), str) and bool(row['captured_at'])
                and isinstance(row.get('evidence_dir'), str) and bool(row['evidence_dir']), '缺少时间/采样证据')
        source = row.get('source', 'serial_capture')
        require(source in ('serial_capture', D6A), '点位来源未知')
        if source == D6A:
            # 上位机动作组目标值；无串口样本，实测反馈留给执行器到位后检查
            d6a = row.get('d6a')
            require(isinstance(d6a, dict) and isinstance(d6a.get('file'), str)
                    and type(d6a.get('frame')) is int and isinstance(d6a.get('sha256'), str)
                    and len(d6a['sha256']) == 64, 'd6a来源信息不完整')
            require(row.get('samples') is None, 'd6a点不得伪造串口样本')
            if color is not None:
                waived = i == 7 and isinstance(row.get('body_motion_waiver'), str) and row['body_motion_waiver'].strip()
                if i in (3, 7) and not waived:
                    require(max(abs(a-b) for a, b in zip(actual[:5], rows[i-2]['encoder_positions'][:5])) <= 12,
                            '夹爪事件的臂体变化超过12刻度')
                if i == 3:
                    close_target = goal
                    require(goal == min(actual[5] + GRIP_SQUEEZE, limits()[5][1]), 'S3夹紧目标须为示教ID6+200')
                elif 4 <= i <= 6:
                    require(goal == close_target and row.get('keep_gripper_goal') is True,
                            '搬运段必须保留第3点夹紧目标')
                else:
                    require(goal == actual[5], '非夹紧点ID6目标须等于示教值')
            continue
        samples = row.get('samples')
        require(isinstance(samples, list) and len(samples) == 8, '每点必须有8个稳定样本')
        for sample in samples:
            require(isinstance(sample, dict), '采样格式错误')
            a, b = counts(sample.get('cmd11_counts')), counts(sample.get('cmd65_counts'))
            require(max(abs(x-y) for x, y in zip(a, b)) <= 6, 'CMD11/65不一致')
            pose = sample.get('controller_pose')
            require(isinstance(pose, dict) and all(type(pose.get(k)) in (int, float)
                    and math.isfinite(pose[k]) for k in ('x', 'y', 'z', 'pitch', 'roll', 'claw')),
                    '缺少控制器位姿，不能把刻度当XYZ')
        for key in ('cmd11_counts', 'cmd65_counts'):
            require(all(max(s[key][j] for s in samples)-min(s[key][j] for s in samples) <= 5
                        for j in range(6)), '样本不稳定')
        require(actual == samples[-1]['cmd65_counts'], '点位与最终样本不符')
        if color is not None:
            if i in (3, 7):
                require(max(abs(a-b) for a, b in zip(actual[:5], rows[i-2]['encoder_positions'][:5])) <= 12,
                        '夹爪事件的臂体变化超过12刻度')
            if i == 3:
                close_target = goal
            if 4 <= i <= 6:
                require(goal == close_target and row.get('keep_gripper_goal') is True,
                        '搬运段必须保留第3点夹紧目标')
    return len(rows)


def init(root, layout_id):
    root = Path(root)
    require(isinstance(layout_id, str) and layout_id.strip(), '需要场地版本名称')
    require(not list(root.rglob('*.json')), '已有JSON数据，拒绝重新初始化；重采请使用新采集目录')
    root.mkdir(parents=True, exist_ok=True)
    scene = dict(schema='nexarm.continuous-scene/1', layout_id=layout_id, created_at=now(),
                 source_colors=list(COLORS), destinations=list(TARGETS),
                 batch_rule='three_distinct_colors_to_P1_P2_P3',
                 initial_occupancy='all_sources_present_and_all_destinations_empty',
                 finish_policy='hold_at_last_point8_until_return_path_is_verified')
    with (root / 'scene.json').open('x', encoding='utf-8') as f:
        json.dump(scene, f, ensure_ascii=False, indent=2)
    for path, color, target in paths(root):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('x', encoding='utf-8') as f:
            json.dump(template(root, color, target), f, ensure_ascii=False, indent=2)
