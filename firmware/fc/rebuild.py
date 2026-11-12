#!/usr/bin/env python3
"""Rebuild the pinned firmware locally. This script has no flashing operation."""
from pathlib import Path
import argparse
import hashlib
import shutil
import subprocess
import tarfile
import urllib.request

HERE = Path(__file__).resolve().parent
BF_SHA = '77d01ba3b76a22909d5f09cb0628820141f95eaa'
CONFIG_SHA = 'bc5da0e66be6d907c694448c3443fdcf6666c1ba'
GCC_ARCHIVE = 'gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2'
GCC_SHA = '97dbb4f019ad1650b732faffcc881689cedc14e2b7ee863d390e0a41ef16c9a3'
GCC_URL = 'https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/' + GCC_ARCHIVE

def run(args, cwd=None):
    subprocess.run([str(x) for x in args], cwd=cwd, check=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work', type=Path, required=True, help='New isolated build directory')
    parser.add_argument('--toolchain', type=Path, help='Existing gcc-arm-none-eabi-10.3-2021.10 directory')
    parser.add_argument('--jobs', type=int, default=6)
    args=parser.parse_args()
    work=args.work.resolve()
    if work.exists():raise SystemExit('Choose a new --work directory; existing builds are not overwritten.')
    work.mkdir(parents=True)
    source=work/'betaflight-4.5.1'
    config=work/'board-config'
    run(['git','clone','--depth','1','--branch','4.5.1','https://github.com/betaflight/betaflight.git',source])
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if actual != BF_SHA:raise SystemExit('Unexpected Betaflight tag revision')
    patch=HERE/'source.patch'
    if not patch.exists():patch=HERE/'package/source.patch'
    run(['git','apply','--check',patch],source)
    run(['git','apply',patch],source)
    config.mkdir()
    run(['git','init','-q',config])
    run(['git','fetch','--depth','1','https://github.com/betaflight/config.git',CONFIG_SHA],config)
    run(['git','checkout','--detach','FETCH_HEAD'],config)
    sdk=args.toolchain.resolve() if args.toolchain else work/'toolchain/gcc-arm-none-eabi-10.3-2021.10'
    if not args.toolchain:
        archive=work/GCC_ARCHIVE
        urllib.request.urlretrieve(GCC_URL,archive)
        if hashlib.sha256(archive.read_bytes()).hexdigest()!=GCC_SHA:raise SystemExit('Toolchain checksum mismatch')
        sdk.parent.mkdir()
        with tarfile.open(archive) as tar:
            tar.extractall(sdk.parent,filter='data')
    version=subprocess.check_output([str(sdk/'bin/arm-none-eabi-gcc'),'-dumpversion'],text=True).strip()
    if version!='10.3.1':raise SystemExit('The pinned GCC 10.3.1 is required')
    binary=work/'firmware';binary.mkdir()
    for kind,options in [('INDI','USE_SANITY_INDI USE_SANITY_SENSOR_STREAM SANITY_GYRO_2KHZ SANITY_INDI_TRACE_DENOM=4 SP_FF_ACCEPT_TRANSFER=1'),('PID_reference','')]:
        out=work/kind;out.mkdir()
        run(['make',f'-j{args.jobs}','hex','CONFIG=BETAFPVF405',f'CONFIG_DIR={config}',
             f'ARM_SDK_DIR={sdk}',f'OBJECT_DIR={work/(kind+"_obj")}',f'BIN_DIR={out}',f'OPTIONS={options}'],source)
        built=out/'betaflight_4.5.1_STM32F405_BETAFPVF405.hex'
        dest=binary/f'Sanity_BETAFPVF405_BF4.5.1_{kind}.hex'
        shutil.copy2(built,dest)
        print(hashlib.sha256(dest.read_bytes()).hexdigest(),dest)
    print('Build complete. No hardware was accessed or flashed.')

if __name__=='__main__':main()
