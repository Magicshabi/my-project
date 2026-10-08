"""Eight calibrated Cartesian poses using the existing CMD8 control path.

Encoder-only teaching records are preserved as provenance, never treated as XYZ.
Controller convergence is not independent physical feedback or collision proof.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time

from .safety import SafetyError

STAGES = ('red_above_open', 'red_down_open', 'red_down_grip', 'red_lift_grip',
          't0_above_grip', 't0_down_grip', 't0_down_open', 't0_clear_open')
AXES = ('x', 'y', 'z', 'pitch', 'roll', 'claw')


def pose_dict(pose):
    return {key: getattr(pose, key) for key in AXES}


def checked_pose(pose):
    if not isinstance(pose, dict) or set(pose) != set(AXES):
        raise SafetyError('需要XYZ/pitch/roll/claw六项标定值；不能用编码器计数代替')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in pose.values()):
        raise SafetyError('位姿必须是有限数值')
    # Check the actual values representable on the command wire.
    result = {k: float(v) for k, v in pose.items()}
    for k in AXES:
        scale = 10 if k == 'pitch' else 1
        result[k] = round(result[k]*scale)/scale
        if not -32768 <= result[k]*scale <= 32767:
            raise SafetyError('位姿超出协议数值范围')
    return result


def template(source):
    source = Path(source)
    record = json.loads(source.read_text(encoding='utf-8-sig'))
    if record.get('complete') is not True or len(record.get('poses', [])) != 8:
        raise SafetyError('原始八点记录不完整')
    points = []
    for idx, row in enumerate(record['poses']):
        if row['stage'] != STAGES[idx] or len(row['encoder_positions']) != 6:
            raise SafetyError('原始八点顺序不匹配')
        points.append(dict(stage=STAGES[idx], legacy_encoders=row['encoder_positions'],
                           pose=None, physical_confirmed=False, observation=''))
    return dict(schema='nexarm.cartesian-eight/1', units='mm,deg',
                source=str(source.resolve()), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                configuration=None, path_observed=False, points=points)


@dataclass(frozen=True)
class Segment:
    point: int
    stage: str
    pose: dict
    duration_ms: int
    dwell_ms: int


def near(a, b, mm=3.0, deg=3.0):
    return all(abs(a[k]-b[k]) <= (mm if k in ('x','y','z') else deg) for k in AXES)


def validate_route(route, limits):
    if route.get('schema') != 'nexarm.cartesian-eight/1' or route.get('units') != 'mm,deg':
        raise SafetyError('八点文件格式或单位不匹配')
    rows = route.get('points', [])
    if len(rows) != 8:
        raise SafetyError('必须恰有八点')
    poses = []
    for i, row in enumerate(rows):
        if row.get('stage') != STAGES[i]:
            raise SafetyError('八点次序错误')
        if row.get('physical_confirmed') is not True or not row.get('observation', '').strip():
            raise SafetyError('第%d点尚未现场核对并记录说明' % (i+1))
        p = checked_pose(row.get('pose'))
        limits.check(**p)
        poses.append(p)
    # Keep approach/lift vertical, and grip/release separate from arm travel.
    for a,b in ((0,1),(2,3),(4,5),(6,7)):
        for k in ('x','y','pitch','roll','claw'):
            if abs(poses[a][k]-poses[b][k]) > 2:
                raise SafetyError('点%d→%d应仅改变Z；需重新核对%s' % (a+1,b+1,k))
    for a,b in ((1,2),(5,6)):
        if any(abs(poses[a][k]-poses[b][k]) > 2 for k in AXES if k != 'claw'):
            raise SafetyError('闭爪/松爪时机械臂位置与朝向应保持不变')
        if abs(poses[a]['claw']-poses[b]['claw']) < 5:
            raise SafetyError('开闭夹爪差值不足，需核对标定')
    for group in ((0,1,6,7),(2,3,4,5)):
        if max(poses[i]['claw'] for i in group)-min(poses[i]['claw'] for i in group) > 2:
            raise SafetyError('张开或夹持阶段的夹爪角度不一致')
    for hi,lo in ((0,1),(3,2),(4,5),(7,6)):
        if poses[hi]['z']-poses[lo]['z'] < 30:
            raise SafetyError('上方/抬离点至少比接触点高30mm，实际避障仍需现场确认')
    if min(poses[3]['z'],poses[4]['z']) < max(poses[2]['z'],poses[5]['z'])+30:
        raise SafetyError('转移段高度不足')
    return poses


def make_plan(route, limits, current):
    poses = validate_route(route, limits)
    current = checked_pose(current)
    # No arbitrary automatic homing from the current pose.
    if not near(current, poses[0], mm=20, deg=5):
        raise SafetyError('起点离第1点过远，先用已验证的上位机安全到达第1点附近')
    result = []
    previous = current
    for i,p in enumerate(poses):
        speed = 10.0 if i in (1,5) else 25.0  # mm/s; gentle contact approaches
        distance = math.dist([previous[k] for k in ('x','y','z')], [p[k] for k in ('x','y','z')])
        angle = max(abs(p[k]-previous[k]) for k in ('pitch','roll'))
        claw = abs(p['claw']-previous['claw'])
        duration = math.ceil(1000*max(0.8, distance/speed, angle/12.0, claw/30.0))
        if duration > 15000:
            raise SafetyError('第%d段超过15秒预算，请重新规划路径而非加速跳过' % (i+1))
        result.append(Segment(i+1,STAGES[i],p,duration,400 if i in (2,6) else 0))
        previous = p
    return result


def configuration(arm):
    limits = arm.get_coord_limits()
    kin = arm.get_kinematics()
    offsets = arm.get_pose_offsets()
    version = arm.get_version()
    if limits is None or kin is None or len(offsets) != 6 or version is None:
        raise SafetyError('无法完整读取限位、机构参数、零偏和版本')
    return dict(limits=list(limits), kinematics=list(kin),
                offsets={str(k):v for k,v in offsets.items()}, version=version)


def same_configuration(expected, actual):
    # JSON normalization makes tuple/list representations comparable.
    return json.dumps(expected,sort_keys=True) == json.dumps(actual,sort_keys=True)


def stable_capture(arm):
    samples = []
    for _ in range(5):
        p = arm.get_pose()
        if p is None:
            raise SafetyError('采集缺少新位姿报告')
        samples.append(checked_pose(pose_dict(p)))
        time.sleep(0.12)
    if any(max(p[k] for p in samples)-min(p[k] for p in samples) > (1 if k != 'pitch' else .5)
           for k in AXES):
        raise SafetyError('采集位姿不稳定；未保存')
    return samples[-1]


def execute(arm, route, segments, confirm, emit, cancelled=lambda: False):
    """Blocking point-boundary control. Never asserts an independently measured arrival.

    Cancellation/timeout stops scheduling only. The current AT32 segment can
    continue; no unverified torque-off, reverse motion or hold command is sent.
    """
    arm.gate.require()
    if confirm is None and route.get('path_observed') is not True:
        raise SafetyError('连贯运行前必须现场验证各段路径')
    if not same_configuration(route.get('configuration'), configuration(arm)):
        raise SafetyError('设备配置与标定记录不一致，禁止继续')
    before = arm.get_pose()
    if before is None or not near(pose_dict(before), segments[0].pose,20,5):
        raise SafetyError('设备起点未确认')
    previous = segments[0].pose
    for index,segment in enumerate(segments):
        if cancelled():
            raise SafetyError('停止后续下发；不代表当前物理运动已停止')
        fresh = arm.get_pose()
        if fresh is None or not near(pose_dict(fresh),previous,20 if index==0 else 3,5 if index==0 else 3):
            raise SafetyError('下一段起点报告缺失或偏离前一点，停止下发')
        emit('command', **segment.__dict__)
        arm.move_to(**segment.pose,time_ms=segment.duration_ms,wait=False)
        # Wait before accepting convergence, even if controller reports a cached goal.
        deadline = time.monotonic()+segment.duration_ms/1000.0
        while time.monotonic() < deadline:
            if cancelled():
                raise SafetyError('停止后续下发；当前AT32运动可能仍在执行')
            time.sleep(.05)
        if not arm.wait_for(segment.pose['x'],segment.pose['y'],segment.pose['z'],
                            pitch=segment.pose['pitch'],roll=segment.pose['roll'],claw=segment.pose['claw'],
                            tol=3,angle_tol=3,timeout=3):
            raise TimeoutError('第%d点控制器报告未收敛，后续点未发送' % segment.point)
        emit('controller_converged', point=segment.point, physical_arrival_verified=False)
        if segment.dwell_ms:
            time.sleep(segment.dwell_ms/1000.0)
        if confirm is not None and not confirm(segment):
            raise SafetyError('现场未确认第%d点；停止后续下发' % segment.point)
        if confirm is not None:
            emit('operator_confirmed', point=segment.point)
        previous = segment.pose
    return dict(controller_sequence_complete=True,
                physical_evidence='operator confirmations' if confirm is not None else 'pending observation',
                measured_accuracy_verified=False)
