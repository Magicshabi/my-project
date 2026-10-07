# -*- coding: utf-8 -*-
"""NexArm 帧编解码 —— 上位机 <-> ESP32(USB/串口) 与 ESP32 <-> AT32(舵机总线) 同构。

帧格式（三处独立来源一致：厂商主机 SDK、ESP32 CommProtocol.cpp、NexArm.exe 反编译）
    +------+------+-----+--------+-----+--------------+----------+
    | 0xFF | 0xFF | ID  | LENGTH | CMD | ARGS (N B)   | CHECKSUM |
    +------+------+-----+--------+-----+--------------+----------+
      ID      : 目标/来源设备号。上位机下发固定 0xFF(广播)；AT32 回包用 0x5A。
      LENGTH  : 2 + len(ARGS)，即从 CMD 到 ARGS 结束的字节数（不含校验）。
      CHECKSUM: (~sum(ID..ARGS)) & 0xFF，注意不含两个 0xFF 帧头。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, List, Tuple

HEADER0 = 0xFF
HEADER1 = 0xFF
SYS_HEAD = 0xFF          # 上位机下发时的目标 ID
AT32_SYS_ID = 0x5A       # AT32 回包时的来源 ID
MAX_ARGS = 250
MAX_FRAME = MAX_ARGS + 5

_START1, _START2, _ID, _LENGTH, _CMD, _DATA, _CHECKSUM = range(7)


def checksum_inv8(data: bytes) -> int:
    """协议校验：对 (ID, LENGTH, CMD, ARGS...) 求和取反，不含帧头。"""
    return (~sum(data)) & 0xFF


def build_frame(cmd: int, args: bytes = b"", target_id: int = SYS_HEAD) -> bytes:
    """组装一个下发帧。args 长度上限 250。"""
    args = bytes(args)
    if len(args) > MAX_ARGS:
        raise ValueError("args too long: %d > %d" % (len(args), MAX_ARGS))
    if not 0 <= cmd <= 0xFF or not 0 <= target_id <= 0xFF:
        raise ValueError("cmd/id out of range")
    body = bytes([target_id, 2 + len(args), cmd]) + args
    return bytes([HEADER0, HEADER1]) + body + bytes([checksum_inv8(body)])


@dataclass(frozen=True)
class Frame:
    id: int
    cmd: int
    args: bytes

    def __len__(self) -> int:
        return len(self.args)


class FrameParser:
    """跨 read() 边界安全的流式解析器。喂多少字节都行，坏帧自动重同步。"""

    def __init__(self) -> None:
        self.reset()
        self.checksum_errors = 0
        self.header_resyncs = 0

    def reset(self) -> None:
        self._state = _START1
        self._id = 0
        self._length = 0
        self._cmd = 0
        self._need = 0
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> List[Frame]:
        out: List[Frame] = []
        for byte in chunk:
            if self._state == _START1:
                if byte == HEADER0:
                    self._state = _START2
            elif self._state == _START2:
                if byte == HEADER1:
                    self._state = _ID
                else:
                    self.header_resyncs += 1
                    self._state = _START1
            elif self._state == _ID:
                self._id = byte
                self._state = _LENGTH
            elif self._state == _LENGTH:
                self._length = byte
                if not 2 <= self._length <= MAX_ARGS + 2:
                    self.header_resyncs += 1
                    self._state = _START1
                else:
                    self._need = self._length - 2
                    self._buf = bytearray()
                    self._state = _CMD
            elif self._state == _CMD:
                self._cmd = byte
                self._state = _DATA if self._need else _CHECKSUM
            elif self._state == _DATA:
                self._buf.append(byte)
                if len(self._buf) >= self._need:
                    self._state = _CHECKSUM
            else:  # _CHECKSUM
                body = bytes([self._id, self._length, self._cmd]) + bytes(self._buf)
                if checksum_inv8(body) == byte:
                    out.append(Frame(self._id, self._cmd, bytes(self._buf)))
                else:
                    self.checksum_errors += 1
                self._state = _START1
        return out

    def iter_frames(self, chunk: bytes) -> Iterator[Tuple[int, int, bytes]]:
        for f in self.feed(chunk):
            yield f.id, f.cmd, f.args
