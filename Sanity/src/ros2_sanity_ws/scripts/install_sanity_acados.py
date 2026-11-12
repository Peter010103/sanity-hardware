#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[4]
REVISION = '503364817c872d474ab5bed219c26760ac267769'


def main():
    parser = argparse.ArgumentParser(description='Install pinned acados C libraries and Python wrappers locally; no sudo.')
    parser.add_argument('--source',type=Path,default=ROOT/'.deps/acados-v0.6.0')
    parser.add_argument('--python',type=Path,action='append',dest='pythons',help='Repeat for each environment; default: this checkout deployment environment')
    parser.add_argument('--jobs',type=int,default=min(8,os.cpu_count() or 1))
    args = parser.parse_args()
    if args.jobs<1:parser.error('--jobs must be positive')
    source = args.source.resolve()
    pythons = args.pythons or [ROOT/(name+'/bin/python') for name in ('.venv-sanity-deploy',)]
    if not all(p.is_file() for p in pythons):parser.error('All Python environments must exist; use --python to select them')
    if not source.exists():
        source.parent.mkdir(parents=True,exist_ok=True)
        subprocess.run(['git','clone','--depth','1','--branch','v0.6.0','https://github.com/acados/acados.git',str(source)],check=True)
    revision = subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    if revision!=REVISION:raise ValueError(f'Expected acados v0.6.0 revision {REVISION}; found {revision}; refusing to replace this checkout')
    subprocess.run(['git','-C',str(source),'submodule','update','--init','--depth','1','external/blasfeo','external/hpipm'],check=True)
    subprocess.run(['cmake','-S',str(source),'-B',str(source/'build'),'-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_INSTALL_PREFIX='+str(source),'-DCMAKE_INSTALL_RPATH='+str(source/'lib'),
        '-DACADOS_WITH_OPENMP=OFF','-DACADOS_EXAMPLES=OFF','-DBUILD_SHARED_LIBS=ON'],check=True)
    subprocess.run(['cmake','--build',str(source/'build'),'--target','install','-j',str(args.jobs)],check=True)
    for python in pythons:
        subprocess.run([str(python.absolute()),'-m','pip','install','casadi==3.7.2','-e',str(source/'interfaces/acados_template')],check=True)
    environment = {**os.environ,'ACADOS_SOURCE_DIR':str(source)}
    subprocess.run([str(pythons[0].absolute()),'-c','from acados_template import get_tera; get_tera()'],env=environment,check=True)
    receipt = {'version':'v0.6.0','revision':revision,'source':str(source),
        'libraries':{name:hashlib.sha256((source/'lib'/name).read_bytes()).hexdigest() for name in ('libacados.so','libhpipm.so','libblasfeo.so')},
        'python_environments':[str(p.absolute()) for p in pythons]}
    (source/'sanity_install.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':main()
