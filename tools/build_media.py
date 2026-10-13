#!/usr/bin/env python3
"""Create compact README media from supplied originals, without changing them.

Requires ffmpeg/ffprobe and Poppler's pdftoppm. Original large videos are outside
this repository; use --source-dir /path/to/original/figures to rebuild previews.
"""
import argparse
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def run(args):
    subprocess.run(args,check=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir',type=Path,required=True)
    args=parser.parse_args();src=args.source_dir;out=ROOT/'media';out.mkdir(exist_ok=True)
    common=['ffmpeg','-hide_banner','-loglevel','error','-y']
    for source,name,start,length in [
        ('15drones_human_4x.mov','swarm',2,4),
        ('LED-8drones.MP4','led',0.5,4)]:
        original=src/source
        video=out/(name+'.mp4');gif=out/(name+'.gif')
        run(common+['-i',str(original),'-map','0:v:0','-an','-map_metadata','-1','-vf','scale=1280:-2',
                    '-c:v','libx264','-preset','medium','-crf','26','-pix_fmt','yuv420p','-movflags','+faststart',str(video)])
        palette='fps=8,scale=480:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=96[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3'
        run(common+['-ss',str(start),'-t',str(length),'-i',str(original),'-an','-map_metadata','-1',
                    '-filter_complex',palette,'-loop','0',str(gif)])
    run(['pdftoppm','-f','1','-singlefile','-scale-to','1400','-png',str(src/'sanity_overview.pdf'),str(out/'overview')])
    run(common+['-i',str(src/'led_drone_flight.jpg'),'-frames:v','1','-vf','scale=1600:-2','-q:v','3','-map_metadata','-1',str(out/'flight.jpg')])
    for f in sorted(out.iterdir()):print(f.name, f.stat().st_size)

if __name__=='__main__':main()
