"""Passive, operator-led teaching validation. NEVER sends serial commands or reset.

Commands: axes, first-save, checkpoint. Saves raw serial data, operator events,
analysis and CRC-checked records under a unique UTC session directory.
No test result certifies torque, electrical health, mechanical limits or replay.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import sys
import threading
import time

from read_teach_record_r6 import decode_snapshot

ROOT = Path(__file__).resolve().parent
FIRMWARE = 'nexarm-teach-record-r7k2'


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def summarize_positions(frames, max_step=128):
    """Conservative review thresholds in raw ticks, NOT hardware angle limits."""
    if len(frames) < 5:
        raise ValueError('少于 5 帧 actual65，不能判断位置')
    for f in frames:
        p = f.get('pulses')
        if (not isinstance(p, list) or len(p) != 6 or
                any(type(v) is not int or not 0 <= v <= 4095 for v in p) or
                type(f.get('ms')) is not int or not 0 <= f['ms'] <= 0xFFFFFFFF):
            raise ValueError('actual65 格式/数值异常')
    for a, b in zip(frames, frames[1:]):
        dt = (b['ms'] - a['ms']) & 0xFFFFFFFF
        if not 0 < dt <= 1000:
            raise ValueError('反馈停顿、时间重复或设备复位，需重新核对')
        if any(abs(x-y) > max_step for x, y in zip(a['pulses'], b['pulses'])):
            raise ValueError('相邻反馈变化过大，需排查实际动作或数据跳变')
    tail = frames[-5:]
    spread = [max(f['pulses'][j] for f in tail) - min(f['pulses'][j] for f in tail) for j in range(6)]
    if max(spread) > 8:
        raise ValueError('末尾五帧未稳定；继续托稳后重测')
    return dict(count=len(frames), median=[statistics.median(f['pulses'][j] for f in tail) for j in range(6)],
                stable_spans=spread, first_ms=frames[0]['ms'], last_ms=frames[-1]['ms'])


def analyze_axis(axis_id, phases, min_delta=4, max_delta=256, max_step=128):
    if axis_id not in range(1, 7):
        raise ValueError('axis_id must be 1..6')
    summaries = {name: summarize_positions(phases[name], max_step) for name in ('before', 'first', 'second', 'returned')}
    # Also check phase boundaries; duplicated boundary frames must not be supplied.
    summarize_positions([f for name in ('before', 'first', 'second', 'returned') for f in phases[name]], max_step)
    med = {k: s['median'] for k, s in summaries.items()}
    j = axis_id - 1
    d1 = med['first'][j] - med['before'][j]
    d2 = med['second'][j] - med['first'][j]
    reasons = []
    if not min_delta <= abs(d1) <= max_delta or not min_delta <= abs(d2) <= max_delta:
        reasons.append('目标轴两次变化幅度未落入本次人工核验窗口，可能未跟随或动作不合适')
    if d1 * d2 <= 0:
        reasons.append('两次同向动作的编码器变化方向不一致')
    if abs(med['returned'][j] - med['before'][j]) > 32:
        reasons.append('回到原姿态后的数值差超过 32 刻度，需核对')
    for k in range(6):
        if k != j and max(abs(med[n][k] - med['before'][k]) for n in ('first', 'second', 'returned')) > 16:
            reasons.append(f'其他 ID{k+1} 也明显变化，不能确认此次单轴对应关系')
    return dict(axis_id=axis_id, passed=not reasons, reasons=reasons, delta_first=d1, delta_second=d2,
                summaries=summaries, thresholds=dict(min_delta=min_delta, max_delta=max_delta, max_step=max_step),
                confirms_only='人工动作与编码器变化的对应关系；不是绝对角度/方向/硬件故障鉴定')


def select_snapshot(frames):
    snapshots = [f for f in frames if f.get('type') == 'teach_record']
    if not snapshots:
        raise ValueError('没有完整示教记录快照')
    # Never fall back to an older success after a newer storage failure.
    decoded, payload = decode_snapshot(snapshots[-1])
    return decoded, payload


class Session:
    def __init__(self, directory):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=False)

    def event(self, event_type, **data):
        with (self.directory / 'events.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(dict(utc=utc_now(), event=event_type, **data), ensure_ascii=False) + '\n')

    def write_json(self, name, value):
        (self.directory / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    def archive_record(self, frames, name):
        decoded, payload = select_snapshot(frames)
        self.write_json(name + '.json', decoded)
        (self.directory / (name + '.bin')).write_bytes(payload)
        self.event('checkpoint', name=name, mask=decoded['valid_mask'], crc32_hex=payload[-4:].hex())
        return decoded, payload


class PassiveReader:
    def __init__(self, port_name, session):
        try:
            import serial
        except ModuleNotFoundError:
            sys.path.insert(0, str(ROOT / '.toolchains' / 'esptool-py-4.5.1'))
            import serial
        self.session = session
        self.port = serial.Serial(port=None, baudrate=1000000, timeout=0.1)
        self.port.port = port_name
        self.port.dtr = False
        self.port.rts = False
        self.frames = []
        self.lines = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.error = None
        self.thread = None

    def __enter__(self):
        self.port.open()
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()
        return self

    def _receive(self):
        pending = bytearray()
        try:
            with (self.session.directory / 'serial.log').open('wb') as log:
                while not self.stop.is_set():
                    chunk = self.port.read(max(1, min(self.port.in_waiting, 8192)))
                    if not chunk:
                        continue
                    log.write(chunk); log.flush()
                    pending.extend(chunk)
                    while b'\n' in pending:
                        line, _, pending = pending.partition(b'\n')
                        text = line.decode('utf-8', errors='replace').strip()
                        with self.lock:
                            self.lines.append(text)
                        try:
                            obj = json.loads(text)
                        except json.JSONDecodeError:
                            continue
                        if isinstance(obj, dict):
                            with self.lock:
                                self.frames.append(obj)
                    if len(pending) > 65536:
                        raise ValueError('串口长时间没有换行，不能继续自动解析')
        except Exception as error:
            self.error = error
            print('\n串口读取中断，请停止手动测试并保持支撑：', error, file=sys.stderr)

    def check(self):
        if self.error:
            raise RuntimeError(f'串口读取失败: {self.error}')

    def mark(self):
        self.check()
        with self.lock:
            return len(self.frames)

    def since(self, index=0, actual_only=False):
        self.check()
        with self.lock:
            result = list(self.frames[index:])
        return [f for f in result if f.get('type') == 'actual65'] if actual_only else result

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check()
            time.sleep(0.05)

    def __exit__(self, *_):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
        self.port.close()


def ask(session, question, confirm=False):
    answer = input(question + (' [输入 YES 确认，其余停止] ' if confirm else ' [完成后回车，异常输入 STOP] ')).strip()
    session.event('operator', question=question, answer=answer)
    if (confirm and answer.upper() != 'YES') or (not confirm and answer):
        raise RuntimeError('人工停止；已保留此前日志')


def capture_phase(reader, session, label, instruction=None):
    begin = reader.mark()
    session.event('phase_start', phase=label)
    if instruction:
        ask(session, instruction)
    reader.wait(2)
    frames = reader.since(begin, actual_only=True)
    session.write_json(label + '.json', frames)
    session.event('phase_end', phase=label, frames=len(frames))
    return frames


def run_axes(reader, session, args, identity):
    results = []
    report = dict(kind='axes', utc=utc_now(), firmware=identity['firmware'], passed=False,
                  axes=results, torque_verified=False, hardware_health_verified=False)
    session.write_json('report.json', report)
    for axis in range(1, 7):
        ask(session, f'ID{axis}：托稳、空载、留有间隙；该关节能轻松移动且无回弹/红灯/异响。夹爪不要求转 10 度', True)
        phases = dict(before=capture_phase(reader, session, f'axis{axis}_before'))
        phases['first'] = capture_phase(reader, session, f'axis{axis}_first',
            f'只将 ID{axis} 缓慢小幅移动，其他轴尽量保持；停稳后回车。有阻力或空间不足则 STOP')
        phases['second'] = capture_phase(reader, session, f'axis{axis}_second',
            f'在仍有安全余量时，将 ID{axis} 再向刚才同一方向小幅移动；停稳后回车，勿追求固定角度')
        phases['returned'] = capture_phase(reader, session, f'axis{axis}_returned',
            f'托稳并轻轻将 ID{axis} 回到本轮开始姿态，停稳后回车；不要硬推到精确刻度')
        try:
            result = analyze_axis(axis, phases, args.min_delta, args.max_delta, args.max_step)
        except ValueError as error:
            result = dict(axis_id=axis, passed=False, reasons=[str(error)])
        results.append(result)
        session.write_json('report.json', report)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if not result['passed']:
            raise RuntimeError(f'ID{axis} 核验未通过，停止后续示教；此结果不等于舵机损坏')
    report['passed'] = True
    session.write_json('report.json', report)


def run_first_save(reader, session, args, identity):
    evidence = json.loads(args.axis_report.read_text(encoding='utf-8'))
    axes = evidence.get('axes', [])
    if (evidence.get('kind') != 'axes' or evidence.get('passed') is not True or
            evidence.get('firmware') != identity['firmware'] or len(axes) != 6 or
            {a.get('axis_id') for a in axes} != set(range(1, 7)) or
            any(a.get('passed') is not True for a in axes)):
        raise ValueError('需同版固件下完整通过的六轴 report.json')
    if identity['valid_mask'] != 0:
        raise ValueError('已有记录；不自动覆盖。先导出并明确是否重教')
    ask(session, '六轴报告属于当前这台机械臂且接线/供电条件未变；两人配合，一人持续托臂观察，一人操作 KEY1；夹爪在提手上方、不接触提手', True)
    before = capture_phase(reader, session, 'save_before')
    start = reader.mark()
    line_start = len(reader.lines)
    ask(session, '托力保持不变，短按 KEY1 后松开；看见 Saved 再回车。任何抽动/红灯/异响立即停止、支撑并切断舵机外接电源，输入 STOP')
    reader.wait(6)
    during = reader.since(start, actual_only=True)
    session.write_json('save_during.json', during)
    after, _ = session.archive_record(reader.since(start), 'first-point')
    a = summarize_positions(before, args.max_step)
    b = summarize_positions(during, args.max_step)
    summarize_positions(before + during, args.max_step)
    with reader.lock:
        lines = list(reader.lines[line_start:])
    saved = any('[TeachRecord] SAVED slot=0 ' in s for s in lines)
    deltas = [y-x for x, y in zip(a['median'], b['median'])]
    drift = [max(abs(f['pulses'][j] - a['median'][j]) for f in during) for j in range(6)]
    ask(session, '观察者确认保存瞬间及随后均无可见抽动、红灯、异响，且全程保持支撑', True)
    passed = saved and after['valid_mask'] == 1 and max(drift) <= 8
    report = dict(kind='first-save', utc=utc_now(), passed=passed, before=a, after=b,
                  saved_event=saved, endpoint_delta=deltas, maximum_observed_drift=drift,
                  observer_no_abnormal_motion=True, torque_verified=False,
                  axis_evidence=str(args.axis_report.resolve()))
    session.write_json('report.json', report)
    if not passed:
        raise RuntimeError('第一点保存观测未通过；检查串口日志，暂不继续后续点')


def run_checkpoint(reader, session, args):
    decoded, payload = session.archive_record(reader.since(), args.stage)
    required = {'first-point': 1, 'eight-points': 255, 'after-restart': 255}[args.stage]
    if decoded['valid_mask'] != required:
        raise ValueError(f'{args.stage} 需要掩码 {required}，实际 {decoded["valid_mask"]}')
    matches = None
    if args.stage == 'after-restart':
        if args.reference is None:
            raise ValueError('after-restart 需要 --reference 指向八点完成时的 .bin')
        matches = payload == args.reference.read_bytes()
        if not matches:
            raise ValueError('重启后记录与重启前不同')
    session.write_json('report.json', dict(kind='checkpoint', utc=utc_now(), passed=True,
        stage=args.stage, valid_mask=decoded['valid_mask'], matches_reference=matches,
        physical_motion_verified=False, playback_validated=False))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=('axes', 'first-save', 'checkpoint'))
    p.add_argument('--port', required=True)
    p.add_argument('--output-root', type=Path, default=ROOT / 'validation_runs')
    p.add_argument('--axis-report', type=Path)
    p.add_argument('--stage', choices=('first-point', 'eight-points', 'after-restart'))
    p.add_argument('--reference', type=Path)
    p.add_argument('--min-delta', type=int, default=4)
    p.add_argument('--max-delta', type=int, default=256)
    p.add_argument('--max-step', type=int, default=128)
    args = p.parse_args()
    if not 0 < args.min_delta <= args.max_delta <= 4095 or not 0 < args.max_step <= 4095:
        p.error('Invalid review thresholds')
    if args.command == 'first-save' and args.axis_report is None:
        p.error('first-save requires --axis-report')
    if args.command == 'checkpoint' and args.stage is None:
        p.error('checkpoint requires --stage')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    session = Session(args.output_root / (stamp + '_' + args.command))
    session.event('start', command=args.command, port=args.port)
    try:
        ask(session, '已亲眼确认 OLED 可读且显示 R7 NO LOCK；机械臂已托稳，无红灯/异响。打开串口可能因驱动导致复位，已做好支撑', True)
        with PassiveReader(args.port, session) as reader:
            reader.wait(6)
            identity, _ = select_snapshot(reader.since())
            if identity['firmware'] != FIRMWARE:
                raise ValueError('本流程需要已修正按键极性的 r7k2；版本不匹配，不继续测试')
            session.archive_record(reader.since(), 'initial')
            if args.command == 'axes':
                run_axes(reader, session, args, identity)
            elif args.command == 'first-save':
                run_first_save(reader, session, args, identity)
            else:
                run_checkpoint(reader, session, args)
        session.event('finished')
        print('本项通过，记录目录：', session.directory)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        session.event('stopped', reason=str(error))
        print('已停止：', error, '\n原始日志已保留：', session.directory)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
