# -*- coding: utf-8 -*-
"""Minimal Python 3.8 marshal reader (enough for PyInstaller PYZ payloads).

Why hand-written: the host has no 3.8 interpreter, and CPython's `marshal` is
not version-stable, so 3.13 cannot load 3.8 bytes. But 3.8's marshal wire
format is simple and fully specified, and 3.8 bytecode has no variable-length
instructions (only an EXTENDED_ARG prefix), so a linear walk is exact.
"""
import struct


class Code:
    __slots__ = ("argcount", "posonlyargcount", "kwonlyargcount", "nlocals",
                 "stacksize", "flags", "code", "consts", "names", "varnames",
                 "freevars", "cellvars", "filename", "name", "firstlineno",
                 "lnotab")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    def __repr__(self):
        return "<Code %s(%s) @%d file=%s>" % (
            self.name, self.argcount, self.firstlineno, self.filename)


class Reader:
    def __init__(self, data):
        self.d = data
        self.p = 0
        self.refs = []

    def byte(self):
        b = self.d[self.p]
        self.p += 1
        return b

    def long(self):
        v = struct.unpack_from("<i", self.d, self.p)[0]
        self.p += 4
        return v

    def long64(self):
        v = struct.unpack_from("<q", self.d, self.p)[0]
        self.p += 8
        return v

    def double(self):
        v = struct.unpack_from("<d", self.d, self.p)[0]
        self.p += 8
        return v

    def bytes_n(self, n):
        v = self.d[self.p:self.p + n]
        self.p += n
        return v

    def _str(self, n, enc=None):
        raw = self.bytes_n(n)
        try:
            return raw.decode(enc or "utf-8")
        except UnicodeDecodeError:
            return raw.decode("latin1")

    def load(self):
        t = self.byte()
        flag = t & 0x80
        t &= 0x7F
        idx = None
        if flag:
            self.refs.append(None)
            idx = len(self.refs) - 1

        def done(v):
            if flag:
                self.refs[idx] = v
            return v

        c = chr(t)
        if c == "0":
            return None
        if c == "N":
            return None
        if c == "F":
            return False
        if c == "T":
            return True
        if c == "S":
            return "StopIteration"
        if c == ".":
            return Ellipsis
        if c == "i":
            return done(self.long())
        if c == "I":
            return done(self.long64())
        if c == "l":
            n = self.long()
            sign = 1
            if n < 0:
                n, sign = -n, -1
            v = 0
            for i in range(n):
                digit = struct.unpack_from("<H", self.d, self.p)[0]
                self.p += 2
                v |= digit << (15 * i)
            return done(sign * v)
        if c == "f":
            return done(_decode_float(self.bytes_n(8)))
        if c == "g":
            return done(self.double())
        if c == "x":
            return done(complex(_decode_float(self.bytes_n(8)),
                                _decode_float(self.bytes_n(8))))
        if c == "y":
            return done(complex(self.double(), self.double()))
        if c in ("s", "a", "A", "t"):
            return done(self._str(self.long(), "utf-8" if c in ("s",) else None))
        if c in ("z", "Z"):
            return done(self._str(self.byte()))
        if c == "u":
            return done(self._str(self.long()))
        if c == "r":
            return self.refs[self.long()]
        if c == "(":
            return done([self.load() for _ in range(self.long())])
        if c == ")":
            return done(tuple(self.load() for _ in range(self.byte())))
        if c == "[":
            return done([self.load() for _ in range(self.long())])
        if c == "{":
            out = {}
            if flag:
                self.refs[idx] = out
            while True:
                pk = self.p
                k = self.load()
                if self.p == pk + 1 and self.d[pk] == 0x30:
                    break
                out[k] = self.load()
            return out
        if c in ("<", ">"):
            n = self.long()
            vals = [self.load() for _ in range(n)]
            return done(set(vals) if c == "<" else frozenset(vals))
        if c == "c":
            return self._code(flag, idx)
        raise ValueError("unsupported marshal type %r (0x%02X) at %d"
                         % (c, t, self.p - 1))

    def _code(self, flag, idx):
        co = Code(
            argcount=self.long(),
            posonlyargcount=self.long(),
            kwonlyargcount=self.long(),
            nlocals=self.long(),
            stacksize=self.long(),
            flags=self.long(),
            code=None, consts=None, names=None, varnames=None,
            freevars=None, cellvars=None, filename=None, name=None,
            firstlineno=None, lnotab=None,
        )
        if flag:
            self.refs[idx] = co
        co.code = _as_bytes(self.load())
        co.consts = tuple(self.load())
        co.names = tuple(self.load())
        co.varnames = tuple(self.load())
        co.freevars = tuple(self.load())
        co.cellvars = tuple(self.load())
        co.filename = self.load()
        co.name = self.load()
        co.firstlineno = self.long()
        co.lnotab = _as_bytes(self.load())
        return co


def _as_bytes(v):
    if isinstance(v, bytes):
        return v
    if isinstance(v, bytearray):
        return bytes(v)
    if isinstance(v, str):
        return v.encode("latin1", "replace")
    return b""


def _decode_float(b):
    return struct.unpack("<d", b)[0]


def loads(data):
    return Reader(data).load()
