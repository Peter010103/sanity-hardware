#!/usr/bin/env python3
"""Check portable links, raw hashes, fitted parameters, PDFs, media and file sizes."""
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest
from urllib.parse import unquote
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

def main():
    raw=json.loads((ROOT/'data/provenance/raw_manifest.json').read_text())
    for item in raw:
        file=ROOT/item['path']
        assert hashlib.sha256(file.read_bytes()).hexdigest()==item['sha256'],item['path']
    bad=[];links=0
    for p in ROOT.rglob('*.md'):
        if any(x in p.parts for x in ['.git','build','.venv']):continue
        text=p.read_text()
        targets=re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',text)+re.findall(r'(?:src|href)="([^"]+)"',text)
        for target in targets:
            if target.startswith(('https://','http://','#','mailto:')):continue
            target=unquote(target.split('#')[0].strip('<>'))
            if not target:continue
            file=(p.parent/target).resolve();links+=1
            if not file.is_file() and not file.is_dir():bad.append((str(p.relative_to(ROOT)),target))
    assert not bad,bad
    media=[]
    for p in (ROOT/'media').rglob('*'):
        if not p.is_file():continue
        if p.suffix in ['.png','.jpg','.gif']:
            with Image.open(p) as im:
                im.load();frames=getattr(im,'n_frames',1)
                if p.suffix=='.gif':assert frames>20
                media.append(dict(file=str(p.relative_to(ROOT)),size=im.size,frames=frames,bytes=p.stat().st_size))
        if p.suffix=='.mp4':
            info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration:stream=codec_type,width,height','-of','json',str(p)],text=True))
            assert float(info['format']['duration'])>5 and any(x.get('codec_type')=='video' for x in info['streams'])
    assert (ROOT/'paper/sanity.pdf').read_bytes().startswith(b'%PDF-')
    assert hashlib.sha256((ROOT/'paper/sanity.pdf').read_bytes()).hexdigest()==json.loads((ROOT/'data/provenance/origin.json').read_text())['paper_sha256']
    model=json.loads((ROOT/'data/processed/actuator/fit_report.json').read_text())
    assert model['fitting_script_sha256']==hashlib.sha256((ROOT/'tools/fit_actuators.py').read_bytes()).hexdigest()
    suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
    tests=unittest.TextTestRunner().run(suite)
    assert tests.wasSuccessful(),'Actuator tests failed'
    eligible=[p for p in ROOT.rglob('*') if p.is_file() and not any(x in p.parts for x in ['.git','build','.venv','__pycache__'])]
    assert all(p.stat().st_size<50_000_000 for p in eligible),'Unexpected large repository file'
    result=dict(raw_hashes_verified=len(raw),relative_links_checked=links,media=media,paper_hash_verified=True,
                evaluator_tests=dict(passed=tests.testsRun-len(tests.skipped),skipped=len(tests.skipped)),
                files=len(eligible),bytes=sum(p.stat().st_size for p in eligible),
                max_file_bytes=max(p.stat().st_size for p in eligible),hardware_access=False)
    out=ROOT/'build';out.mkdir(exist_ok=True)
    (out/'repository_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='media'},indent=2))

if __name__=='__main__':main()
