# -*- coding: utf-8 -*-
"""传输层：串口（USB）或 WiFi(TCP) <-> NexArm，带后台收包线程与命令订阅。

设计要点（对应我们踩过的坑）：
  * 只允许一条线程写串口 -> tx_lock，避免半帧交错。
  * 读线程独立，自己维护缓冲与重同步 -> 不依赖 readline/超时节奏。
  * 逐命令订阅（on(cmd, cb)），不把解析结果塞进全局队列后到处轮询。
  * 连接用"代次"(generation) 保护，避免上一轮的收包线程把新连接的 buffer 弄脏。
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Dict, List, Optional

from .frames import AT32_SYS_ID, SYS_HEAD, Frame, FrameParser, build_frame

try:  # pyserial 只在真正连串口时才需要
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None  # type: ignore

DEFAULT_BAUDRATE = 1_000_000
Callback = Callable[[Frame], None]


class TransportError(RuntimeError):
    pass


class Transport:
    def __init__(self, log: Optional[Callable[[str], None]] = None) -> None:
        self._log = log or (lambda msg: None)
        self._ser = None
        self._sock = None
        self._parser = FrameParser()
        self._subs: Dict[int, List[Callback]] = {}
        self._any: List[Callback] = []
        self._tx_lock = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._gen = 0
        self.connection_type: Optional[str] = None
        self.rx_frames = 0
        self.rx_bytes = 0
        self.tx_frames = 0

    # ---------------------------------------------------------------- connect
    @property
    def connected(self) -> bool:
        return bool(self._ser is not None or self._sock is not None)

    def open_serial(self, port: str, baudrate: int = DEFAULT_BAUDRATE,
                    timeout: float = 0.02, write_timeout: float = 0.2) -> None:
        if serial is None:
            raise TransportError("pyserial 未安装：pip install pyserial")
        self.close()
        try:
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout,
                                write_timeout=write_timeout)
        except PermissionError as exc:
            raise TransportError(
                "打开 %s 被拒绝：串口多半被 NexArm.exe 等上位机占用，先关掉它"
                % port) from exc
        except Exception as exc:
            raise TransportError("打开 %s 失败: %s" % (port, exc)) from exc
        ser.dtr = False
        ser.rts = False
        self._ser = ser
        self.connection_type = "serial"
        time.sleep(0.05)
        ser.reset_input_buffer()
        self._start_rx()
        self._log("已连接 %s @ %d" % (port, baudrate))

    def open_socket(self, host: str, port: int = 8080, timeout: float = 5.0) -> None:
        import socket as _socket
        self.close()
        sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.settimeout(0.02)
        self._sock = sock
        self.connection_type = "wifi"
        self._start_rx()
        self._log("已连接 %s:%d" % (host, port))

    def _start_rx(self) -> None:
        self._stop.clear()
        self._parser.reset()
        self._gen += 1
        gen = self._gen
        self._rx_thread = threading.Thread(target=self._rx_loop, args=(gen,),
                                           daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop.set()
        self._gen += 1
        thread, self._rx_thread = self._rx_thread, None
        if thread and thread.is_alive():
            thread.join(timeout=0.5)
        for attr in ("_ser", "_sock"):
            obj = getattr(self, attr)
            if obj is not None:
                try:
                    obj.close()
                except Exception:
                    pass
                setattr(self, attr, None)
        self.connection_type = None

    # -------------------------------------------------------------- rx / subs
    def _rx_loop(self, gen: int) -> None:
        idle = 0.005
        while not self._stop.is_set() and gen == self._gen:
            try:
                chunk = b""
                if self._ser is not None:
                    n = getattr(self._ser, "in_waiting", 0)
                    if n:
                        chunk = self._ser.read(n)
                elif self._sock is not None:
                    try:
                        chunk = self._sock.recv(1024)
                    except Exception:
                        chunk = b""
                        time.sleep(idle)
                        continue
                    if not chunk:
                        self._log("远端关闭了连接")
                        self._stop.set()
                        break
                if not chunk:
                    time.sleep(idle)
                    continue
                self.rx_bytes += len(chunk)
                for frame in self._parser.feed(chunk):
                    self.rx_frames += 1
                    self._dispatch(frame)
            except Exception as exc:  # 串口被拔 -> 抛错，退出线程
                if gen == self._gen:
                    self._log("接收线程异常: %s" % exc)
                break

    def _dispatch(self, frame: Frame) -> None:
        if frame.id not in (SYS_HEAD, AT32_SYS_ID):
            self._log("忽略非本机回包 id=0x%02X cmd=%d" % (frame.id, frame.cmd))
        for cb in list(self._subs.get(frame.cmd, ())):
            try:
                cb(frame)
            except Exception as exc:
                self._log("订阅回调异常 cmd=%d: %s" % (frame.cmd, exc))
        for cb in list(self._any):
            try:
                cb(frame)
            except Exception as exc:
                self._log("通配回调异常: %s" % exc)

    def on(self, cmd: int, callback: Callback) -> Callback:
        """订阅某个命令号的回包；返回 callback 便于 off()。"""
        self._subs.setdefault(int(cmd), []).append(callback)
        return callback

    def on_any(self, callback: Callback) -> Callback:
        self._any.append(callback)
        return callback

    def off(self, cmd: int, callback: Callback) -> None:
        lst = self._subs.get(int(cmd))
        if lst and callback in lst:
            lst.remove(callback)

    # -------------------------------------------------------------------- tx
    def send(self, cmd: int, args: bytes = b"", target_id: int = SYS_HEAD) -> None:
        if not self.connected:
            raise TransportError("未连接")
        data = build_frame(cmd, args, target_id)
        with self._tx_lock:
            if self._ser is not None:
                self._ser.write(data)
            elif self._sock is not None:
                self._sock.sendall(data)
        self.tx_frames += 1

    def request(self, cmd: int, args: bytes = b"", reply_cmd: Optional[int] = None,
                timeout: float = 0.3) -> Optional[Frame]:
        """发一条命令并等它的回包（同步语义，供状态查询使用）。"""
        reply_cmd = cmd if reply_cmd is None else reply_cmd
        box: List[Frame] = []
        done = threading.Event()

        def _cb(frame: Frame) -> None:
            box.append(frame)
            done.set()

        self.on(reply_cmd, _cb)
        try:
            self.send(cmd, args)
            if done.wait(timeout):
                return box[0]
            return None
        finally:
            self.off(reply_cmd, _cb)

    def stats(self) -> dict:
        return {"connected": self.connected, "type": self.connection_type,
                "rx_frames": self.rx_frames, "rx_bytes": self.rx_bytes,
                "tx_frames": self.tx_frames,
                "checksum_errors": self._parser.checksum_errors,
                "resyncs": self._parser.header_resyncs}
