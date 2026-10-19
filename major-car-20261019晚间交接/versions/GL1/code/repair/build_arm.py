"""Build repo sources with the installed ArmClang, without installing CMSIS Packs.

Reuses the checked-in project file list, defines, includes and scatter layout.
Does not flash. The compiler --version is saved in each build log. T1 was built
with Arm Compiler 6.21; archived T/N files used 6.7 and are not byte-identical.
"""
from pathlib import Path
import argparse
import json
import os
import re
import shlex
import subprocess
import xml.etree.ElementTree as ET

base=Path(__file__).resolve().parents[1]
firmware=base/'competition/firmware/camera_car'
mdk=firmware/'demo/mdk'
bin_dir=Path(r'D:\keil5\ARM\ARMCLANG\bin')
parser=argparse.ArgumentParser()
parser.add_argument('--mode',choices=['camera','diag','mock'],default='diag')
args=parser.parse_args()
out=base/'repair/build'/args.mode
out.mkdir(parents=True,exist_ok=True)
tree=ET.parse(mdk/'rt1064.uvprojx')
target=tree.find('.//Target')
c=target.find('.//Cads/VariousControls')
defines=[re.sub(r'\s+','',v) for v in c.findtext('Define').split(',') if v.strip()]
defines+=['__MICROLIB','__UVISION_VERSION=524','CAR_GROUND_DIAGNOSTIC=1','CAR_GROUND_CLOSED_DIAGNOSTIC=1']
if args.mode=='diag': defines+=['CAR_COMM_DIAGNOSTIC=1']
if args.mode=='mock': defines+=['CAR_SIMULATED=1']
includes=[str((mdk/p.replace('\\','/')).resolve()) for p in c.findtext('IncludePath').split(';') if p]
common=['--target=arm-arm-none-eabi','-mcpu=cortex-m7','-mfpu=fpv5-d16','-mfloat-abi=hard','-mthumb',
        '-g','-Oz','-ffunction-sections','-fdata-sections','-fno-common','-ffreestanding','-fno-builtin',
        '-fshort-enums','-fshort-wchar','-Wno-invalid-source-encoding']
common+=['-D'+v for v in defines]
common+=['-I'+v for v in includes]
objects=[]
libs=[]
commands=[]
with (out/'build.log').open('w',encoding='utf-8') as log:
    def run(cmd):
        commands.append([str(s) for s in cmd])
        result=subprocess.run(cmd,cwd=mdk,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        try: text=result.stdout.decode('utf-8')
        except UnicodeDecodeError: text=result.stdout.decode('mbcs',errors='replace')
        log.write(subprocess.list2cmdline([str(s) for s in cmd])+'\n'+text);log.flush()
        if result.returncode:
            print(text[-6000:]);raise SystemExit(result.returncode)
    run([str(bin_dir/'armclang.exe'),'--version'])
    for item in target.findall('./Groups/Group/Files/File'):
        if item.findtext('./FileOption/CommonProperty/IncludeInBuild')=='0': continue
        source=(mdk/item.findtext('FilePath').replace('\\','/')).resolve()
        if source.suffix.lower() in ('.lib','.a'):
            libs.append(str(source));continue
        if source.suffix.lower() not in ('.c','.s'): continue
        obj=out/(source.stem+'.o')
        if obj in objects: raise RuntimeError('Duplicate object name: '+str(obj))
        command=[str(bin_dir/'armclang.exe')]+common
        if source.suffix.lower()=='.s':
            command+=['-x','assembler-with-cpp','-D__STARTUP_INITIALIZE_NONCACHEDATA']
        else: command+=['-std=c99']
        run(command+['-c',str(source),'-o',str(obj)])
        objects.append(obj)
    axf=out/('camera-car-uart8-'+args.mode+'.axf')
    run([str(bin_dir/'armlink.exe'),'--cpu=Cortex-M7.fp.dp','--library_type=microlib',
         '--scatter='+str(mdk/'scf/MIMXRT1064xxxxx_flexspi_nor.scf'),
         '--entry=Reset_Handler','--remove','--map','--info=sizes','--diag_suppress=6314,6329',
         '--list='+str(out/'link.map'),'--output='+str(axf)]+[str(o) for o in objects]+libs)
    run([str(bin_dir/'fromelf.exe'),'--i32combined','--output='+str(axf.with_suffix('.hex')),str(axf)])
    log.write('BUILD SUCCESS\n')
(out/'commands.json').write_text(json.dumps(commands,indent=2),encoding='utf-8')
print('BUILD SUCCESS: '+str(axf.with_suffix('.hex')))
