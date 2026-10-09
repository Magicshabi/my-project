"""Read ID4 registers once; never enable torque, move, or change settings.

All replies are logged, regardless of the fifth byte's value. A native servo
status packet may use that byte as status, not echo CMD_READ. No reply is not
proof of torque OFF. Even a present reply cannot reconstruct the earlier run.
"""
import json
from pathlib import Path
import threading
import time

from capture_joint_points import ArmController, ReadOnlyTransport, timestamp, digest
from nexarm import Transport, SafetyError, build_frame
from nexarm import transport as transport_module

ROOT = Path(__file__).resolve().parent
RAW_READS = (bytes([40, 8]), bytes([56, 2]))


class RegisterQueries(ReadOnlyTransport):
    def __init__(self, emit):
        super().__init__(emit)
        self.used = set()

    def send(self, cmd, args=b'', target_id=255):
        args = bytes(args)
        if target_id == 4 and cmd == 2 and args in RAW_READS:
            if args in self.used:
                raise SafetyError('只读寄存器已查询过；不重试')
            self.used.add(args)
            self.emit('tx_attempt', cmd=cmd, id=target_id,
                      wire_hex=build_frame(cmd, args, target_id).hex())
            Transport.send(self, cmd, args, target_id)
            self.emit('tx_complete', cmd=cmd, id=target_id)
        elif target_id == 255 and cmd in (1, 2, 11, 65) and not args:
            super().send(cmd, args, target_id)
        else:
            raise SafetyError('只允许ID4已核对寄存器及系统状态查询；拒绝任何写入')

    def open_serial(self, port, baudrate=1000000, timeout=.02, write_timeout=.2):
        # Configure lines before opening; do not deliberately pulse EN/BOOT.
        self.close()
        ser = transport_module.serial.Serial(port=None, baudrate=baudrate,
                                              timeout=timeout, write_timeout=write_timeout)
        try:
            ser.dtr = False
            ser.rts = False
            ser.port = port
            ser.open()
            self._ser = ser
            self.connection_type = 'serial'
            time.sleep(.05)
            ser.reset_input_buffer()
            self._start_rx()
        except BaseException:
            ser.close()
            self._ser = None
            self.connection_type = None
            raise


def main():
    run = ROOT/'logs'/('id4_register_read_'+str(time.time_ns()))
    run.mkdir(parents=True)
    result = dict(started_at=timestamp(), read_only=True, motion_commands=0,
                  torque_writes=0, configuration_writes=0, raw_reads=[],
                  torque_state_verified=False, source_sha256=digest(__file__),
                  note='Unknown reply formats are preserved, not interpreted as measured registers.')
    print('RUN_DIR='+str(run), flush=True)
    lock = threading.Lock()
    with (run/'events.jsonl').open('w', encoding='utf-8') as log:
        replies = []
        def emit(event, **data):
            with lock:
                log.write(json.dumps(dict(event=event,t=time.monotonic(),**data),ensure_ascii=False)+'\n')
                log.flush()
        def received(frame):
            entry = dict(arrived_monotonic=time.monotonic(),id=frame.id,cmd_or_status=frame.cmd,
                         args_hex=frame.args.hex(),args_length=len(frame.args))
            with lock:
                replies.append(entry)
            emit('rx', **entry)
        t = RegisterQueries(emit)
        t.on_any(received)
        arm = ArmController(transport=t,log=lambda s:emit('library',message=s))
        try:
            arm.connect_serial('COM9')
            version = arm.get_version()
            result['version'] = version
            if version is None or version.get('esp_str')!='1.0.2' or version.get('at32_str')!='1.0.42':
                raise SafetyError('固件版本未知或已变化，不继续寄存器探查')
            before = arm.get_real_joints()
            if before is None:
                raise SafetyError('无CMD65反馈，不继续查询')
            result['before'] = [v[0] for v in before]
            for args in RAW_READS:
                begun = time.monotonic()
                t.send(2, args, target_id=4)
                time.sleep(.7)
                with lock:
                    seen = [v.copy() for v in replies if v['arrived_monotonic'] >= begun]
                result['raw_reads'].append(dict(register=args[0],requested_bytes=args[1],
                    replies=seen, matching_id4_replies=[v for v in seen if v['id']==4]))
                if t.stats()['checksum_errors'] or not t.connected:
                    raise SafetyError('通信异常，停止查询')
            after = arm.get_real_joints()
            result['after'] = [v[0] for v in after] if after else None
        except BaseException as exc:
            result['error'] = str(exc) or type(exc).__name__
        finally:
            arm.close()
            result.update(stats=t.stats(),serial_closed=True)
            (run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
