"""Extract selected embedded Python modules for static protocol inspection.

Does not launch NexArm.exe or execute extracted code.
"""
from pathlib import Path
import hashlib
import json
import marshal
import struct
import zlib

exe = Path('A:/NEXarm/NexArm.exe')
b = exe.read_bytes()
k = b.rfind(b'MEI\014\013\012\013\016')
magic, size, off, length, pyver, lib = struct.unpack('!8sIIII64s', b[k:k+88])
assert pyver == 308
start = k+88-size
table = b[start+off:start+off+length]
entries = {}
i = 0
while i < len(table):
    n, pos, compressed, raw, flag, typ = struct.unpack('!iIIIBB', table[i:i+18])
    assert n >= 18
    name = table[i+18:i+n].rstrip(b'\0').decode()
    entries[name] = (pos, compressed, raw, flag, typ)
    i += n
pos, size, _, _, _ = entries['PYZ.pyz']
pyz = b[start+pos:start+pos+size]
assert pyz[:4] == b'PYZ\0'
toc = dict(marshal.loads(pyz[struct.unpack('!I', pyz[8:12])[0]:]))
dest = Path(__file__).resolve().parent/'nexarm_exe_static'
dest.mkdir(exist_ok=True)
(dest/'modules.json').write_text(json.dumps(sorted(toc), indent=2))
selected = []
for name, (kind, pos, size) in toc.items():
    if name.startswith(('ui', 'sdk', 'nex', 'robot', 'protocol', 'controller', 'utils', 'serial_manager')):
        if kind not in (0, 1):
            continue
        raw = zlib.decompress(pyz[pos:pos+size])
        assert '/' not in name and '\\' not in name
        (dest/(name+'.pyc')).write_bytes(pyz[4:8]+bytes(12)+raw)
        selected.append(name)
manifest = dict(exe_sha256=hashlib.sha256(b).hexdigest(), python=pyver,
                selected=selected, module_count=len(toc), code_executed=False)
(dest/'manifest.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps(manifest, indent=2))
