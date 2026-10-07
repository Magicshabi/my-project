"""Single ID6 probe matching NexArm.exe's static send_servo_cmd implementation.

Same -8 count target and feedback checks as official_replay. Uses the EXE's
default acceleration=100/speed=2000; no torque/configuration writes or retries.
Not an eight-point replay entrypoint.
"""
import struct
import sys
import official_replay as replay


def exe_frame(sid, position):
    if sid != 6 or type(position) is not int or not 1100 <= position <= 1900:
        raise replay.Refused('EXE comparison probe restricted to ID6 interior')
    payload = struct.pack('<BBhHH', 41, 100, position, 0, 2000)
    body = bytes((sid, len(payload)+2, 3))+payload
    return b'\xff\xff'+body+bytes(((~sum(body)) & 255,))


class ExeProbeDevice(replay.Device):
    def send(self, cmd, payload=b''):
        if cmd != 51:
            return super().send(cmd, payload)
        # Validate the inherited small-step request before translating it.
        replay.frame(cmd, payload)
        sid, position, _ = struct.unpack('<BhH', payload)
        b = exe_frame(sid, position)
        self.motion_sent = True
        self.touched.add(sid)
        self.emit('tx_attempt', cmd=3, servo_id=sid, position=position,
                  acceleration=100, speed=2000, hex=b.hex(),
                  interface='NexArm.exe direct write reg41')
        try:
            n = self.s.write(b)
            if n != len(b):
                raise replay.Refused('Short UART write; no retry')
        except BaseException:
            self.tx_failed = True
            raise
        self.emit('tx_complete', cmd=3, bytes=n, acknowledged=False)


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] != 'probe':
        raise SystemExit('Only probe --execute is supported')
    replay.Device = ExeProbeDevice
    replay.HERE = replay.HERE/'exe_interface_probe'
    replay.main()
