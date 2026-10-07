"""Offline decoder for NexArm.exe exported [TX]/[RX] hexadecimal logs."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import struct


def decode_text(text):
    buffers = {'TX': bytearray(), 'RX': bytearray()}
    frames = []
    bad = 0
    for line_no, line in enumerate(text.splitlines(), 1):
        m = re.search(r'\[(TX|RX)\]\s*([0-9A-Fa-f\s]+)$', line)
        if not m:
            continue
        direction = m[1]
        buf = buffers[direction]
        buf.extend(bytes.fromhex(m[2]))
        while len(buf) >= 4:
            i = buf.find(b'\xff\xff')
            if i < 0:
                buf[:] = buf[-1:] if buf[-1:] == b'\xff' else b''
                break
            del buf[:i]
            if len(buf) < 4:
                break
            if buf[2] not in (1,2,3,4,5,6,90,254,255) or not 2 <= buf[3] <= 252:
                del buf[0]; bad += 1; continue
            n = buf[3]+4
            if len(buf) < n:
                break
            raw = bytes(buf[:n])
            if sum(raw[2:]) & 255 != 255:
                del buf[0]; bad += 1; continue
            del buf[:n]
            sid, cmd, payload = raw[2], raw[4], raw[5:-1]
            row = dict(line=line_no,direction=direction,id=sid,cmd=cmd,hex=raw.hex())
            if direction == 'TX' and 1 <= sid <= 6 and cmd == 3 and len(payload) == 8 and payload[0] == 41:
                reg,acc,pos,pwm,speed = struct.unpack('<BBHHH',payload)
                row.update(kind='joint_target',acceleration=acc,position=pos,pwm=pwm,speed=speed)
            elif direction == 'TX' and cmd == 3 and len(payload) == 2 and payload[0] == 40:
                row.update(kind='torque_request',enable=payload[1])
            elif direction == 'RX' and cmd == 65 and len(payload) == 24:
                row.update(kind='actual65',positions=[struct.unpack_from('<h',payload,i*4)[0] for i in range(6)])
            elif direction == 'RX' and cmd == 1:
                row.update(kind='version',payload=list(payload))
            else:
                row.update(kind='other',payload=payload.hex())
            frames.append(row)
    return dict(frames=frames,bad_headers_or_checksums=bad,
                physical_success='requires operator observation; decoding is not proof of movement')


if __name__ == '__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('path',type=Path)
    a=ap.parse_args()
    raw=a.path.read_bytes()
    text=raw.decode('utf-16') if raw[:2] in (b'\xff\xfe',b'\xfe\xff') else raw.decode('utf-8-sig')
    result=decode_text(text)
    a.path.with_suffix('.decoded.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(counts=dict(Counter(r['kind'] for r in result['frames'])),
                         control_frames=[r for r in result['frames'] if r['kind'] in ('joint_target','torque_request','version')],
                         bad=result['bad_headers_or_checksums']),indent=2))
