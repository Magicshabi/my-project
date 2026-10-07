"""Offline validation and extraction of the pre-restore full flash backup."""
import hashlib
import json
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1]
DEST = Path(__file__).resolve().parent / 'backups/before_motion_restore_20261007'
blob = (DEST / 'flash_full_4MB.bin').read_bytes()
assert len(blob) == 0x400000, 'Full flash read not complete'
entries = []
for off in range(0x8000, 0x8C00, 32):
    if blob[off:off+2] != b'\xaa\x50':
        break
    magic, typ, sub, start, size, name, flags = struct.unpack('<HBBII16sI', blob[off:off+32])
    assert start+size <= len(blob)
    entries.append(dict(type=typ, subtype=sub, offset=start, size=size,
                        name=name.split(b'\0')[0].decode('ascii'), flags=flags))
assert entries, 'No valid partition entries'
nvs = next(p for p in entries if p['type'] == 1 and p['subtype'] == 2)
app = next(p for p in entries if p['type'] == 0 and p['offset'] == 0x10000)
candidate = (ROOT/'factory_app_recovered.bin').read_bytes()
assert hashlib.sha256(candidate).hexdigest() == '8dbe55e19f495e9469fe945aca1ad19757991501a6cbf208af3b5f91bdf59dc9'
assert len(candidate) <= app['size'], 'Current app partition too small; do not flash'
assert nvs['offset'] == 0x9000 and nvs['size'] == 0x5000
files = {
    'nvs_before.bin': blob[nvs['offset']:nvs['offset']+nvs['size']],
    'partitions_before.bin': blob[0x8000:0x9000],
    'official_motion_candidate.bin': candidate,
    'eight_points.bin': (ROOT/'validation_runs/readout_20261005_180008/eight_points.bin').read_bytes(),
}
hashes = {'flash_full_4MB.bin': hashlib.sha256(blob).hexdigest()}
for name, data in files.items():
    (DEST/name).write_bytes(data)
    hashes[name] = hashlib.sha256(data).hexdigest()
manifest = dict(port='COM9', chip_mac='68:09:47:7b:9f:64', bytes=len(blob),
                partitions=entries, sha256=hashes, device_backup_verified=False,
                candidate_internal_checksum_verified=True, flash_performed=False,
                note='Host SHA only until esptool verify_flash confirms device equality; no erase or partition change planned.')
(DEST/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
print(json.dumps(manifest, indent=2))
