#!/usr/bin/env python3
"""Verify the standalone source payload without network or hardware access."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def main():
    manifest=json.loads((ROOT/'PACKAGED_FILES.json').read_text());errors=[]
    for name,expected in manifest['files'].items():
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            errors.append(name+': invalid payload path');continue
        path=ROOT/relative
        try:data=str(path.readlink()).encode() if path.is_symlink() else path.read_bytes()
        except OSError as exc:errors.append(name+': '+str(exc));continue
        if len(data)!=expected['bytes'] or hashlib.sha256(data).hexdigest()!=expected['sha256']:
            errors.append(name+': content differs from manifest')
    if errors:raise SystemExit('\n'.join(errors))
    print(f'Verified {len(manifest["files"])} packaged files; no network or devices accessed.')

if __name__=='__main__':main()
