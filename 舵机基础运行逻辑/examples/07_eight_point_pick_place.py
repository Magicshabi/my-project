"""八点CMD8适配：create/plan完全离线，capture只读，run --go才下发运动。"""
import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import sys
import time

LIB = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(LIB))
from nexarm import ArmController
from nexarm.eight_point import (STAGES, AXES, template, checked_pose, configuration,
    same_configuration, stable_capture, make_plan, execute, pose_dict)
from nexarm.safety import SafetyError, WorkspaceLimits


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.with_name(path.name+'.'+str(time.time_ns())+'.bak').write_bytes(path.read_bytes())
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode',choices=('create','capture','plan','run'))
    ap.add_argument('--file',type=Path,default=LIB/'data/eight_cartesian.json')
    ap.add_argument('--source',type=Path,default=LIB.parent/'validation_runs/readout_20261005_180008/eight_points.json')
    ap.add_argument('--port',default='COM9')
    ap.add_argument('--point',type=int,choices=range(1,9))
    ap.add_argument('--observed',help='亲眼核对该点与控制器显示一致的描述；手拖后只读缓存不足以标定')
    ap.add_argument('--current',type=float,nargs=6,metavar='VALUE')
    ap.add_argument('--go',action='store_true')
    ap.add_argument('--continuous',action='store_true')
    ap.add_argument('--path-reviewed',action='store_true',help='操作者已实际逐段验证此文件中的路线')
    a=ap.parse_args()
    if a.mode=='create':
        if a.file.exists(): raise SafetyError('文件已存在；不会覆盖已有标定')
        save(a.file,template(a.source)); print('已保留八点计数，待补齐八个完整坐标：',a.file); return
    route=json.loads(a.file.read_text(encoding='utf-8-sig'))
    if a.mode=='plan':
        if route.get('configuration') is None: raise SafetyError('尚无本机标定配置，先逐点capture')
        limits=WorkspaceLimits(); limits.from_payload(route['configuration']['limits'])
        current=dict(zip(AXES,a.current)) if a.current else route['points'][0]['pose']
        steps=make_plan(route,limits,current)
        result=dict(assumes_start_at_point1=a.current is None,segments=[asdict(s) for s in steps],
                    minimum_planned_seconds=sum(s.duration_ms+s.dwell_ms for s in steps)/1000,
                    physical_path_validated=False)
        save(a.file.with_suffix('.plan.json'),result); print(json.dumps(result,ensure_ascii=False,indent=2)); return
    if a.mode=='capture' and (a.point is None or not a.observed or not a.observed.strip()):
        ap.error('capture需要--point 1..8和--observed 现场复核说明')
    if a.mode=='run' and not a.go: ap.error('run需要--go；先用plan离线核对')
    if a.continuous and not a.path_reviewed: ap.error('连贯运行需要先逐段验证，再使用--path-reviewed')
    arm=ArmController()
    try:
        arm.connect_serial(a.port)
        live=configuration(arm)
        if a.mode=='capture':
            if route.get('configuration') is not None and not same_configuration(route['configuration'],live):
                raise SafetyError('配置与已有点位不一致，需要重新核对标定')
            p=stable_capture(arm); arm.limits.check(**p)
            row=route['points'][a.point-1]
            row.update(pose=p,physical_confirmed=True,observation=a.observed,
                       captured_at=datetime.now().astimezone().isoformat(),source='CMD11 + operator confirmation')
            route['configuration']=live; route['path_observed']=False
            save(a.file,route); print('已保存第%d点 %s：%s' % (a.point,row['stage'],p)); return
        if not same_configuration(route.get('configuration'),live):
            raise SafetyError('当前配置与标定文件不一致')
        current=arm.get_pose()
        if current is None: raise SafetyError('缺少当前位姿报告')
        steps=make_plan(route,arm.limits,pose_dict(current))
        route['path_observed']=bool(a.path_reviewed)
        run_dir=LIB/'logs'/('eight_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:])
        run_dir.mkdir(parents=True)
        (run_dir/'route.json').write_text(json.dumps(route,ensure_ascii=False,indent=2),encoding='utf-8')
        print('运行日志：',run_dir,flush=True)
        with (run_dir/'events.jsonl').open('w',encoding='utf-8') as log:
            def emit(kind,**data):
                row=dict(type=kind,monotonic=time.monotonic(),**data)
                log.write(json.dumps(row,ensure_ascii=False)+'\n'); log.flush()
                print(json.dumps(row,ensure_ascii=False),flush=True)
            def confirm(segment):
                return input('第%d点控制器报告已收敛。确认物理到位且无异常，输入 yes 继续：' % segment.point).strip()=='yes'
            arm.enable_motion(True)
            try:
                result=execute(arm,route,steps,None if a.continuous else confirm,emit,
                               cancelled=lambda:(run_dir/'STOP').exists())
                emit('complete',**result)
            except (Exception,KeyboardInterrupt) as exc:
                emit('aborted',reason=str(exc) or 'KeyboardInterrupt',physical_stop_verified=False)
                print('已停止后续下发；当前AT32段可能继续。若仍运动或出现异常，请托稳并断舵机电源。')
                raise
            finally:
                arm.enable_motion(False)
    finally:
        arm.close()


if __name__=='__main__':
    try: main()
    except (SafetyError,TimeoutError) as exc:
        print('未执行/已中止：',exc,file=sys.stderr); raise SystemExit(2)
