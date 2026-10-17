"""Back up RT1064 flash, program motion-disabled diagnostic, read back and verify.

Requires the user's confirmed battery-off, DAP-connected state. Uses only this
task's local pyOCD install. Does not resume the core on disconnect.
"""
from pathlib import Path
import hashlib
import json
import sys
import os
import time
import argparse
base=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--frequency',type=int,default=1000000)
parser.add_argument('--connect',choices=['halt','under-reset','attach'],default='halt')
parser.add_argument('--nxp-flm',action='store_true')
parser.add_argument('--reuse-backup',action='store_true')
parser.add_argument('--mode',choices=['diag','camera','mock'],default='diag')
args=parser.parse_args()
sys.path.insert(0,str(base/'repair/tools'))
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from intelhex import IntelHex
from pyocd.target.pack.flash_algo import PackFlashAlgo
from pyocd.core.memory_map import FlashRegion,RamRegion

evidence=base/'repair/evidence'
image=base/'repair/build'/args.mode/('camera-car-uart8-'+args.mode+'.hex')
backup=evidence/'before-flash-70000000-4MiB.bin'
backup_meta=evidence/'before-flash-manifest.json'
if args.reuse_backup:
    previous=json.loads(backup_meta.read_text(encoding='utf-8'))
    original=backup.read_bytes()
    if len(original)!=0x400000 or original[:4]!=b'FCFB':
        raise RuntimeError('No complete original backup to reuse')
    if hashlib.sha256(original).hexdigest()!=previous['sha256']:
        raise RuntimeError('Original backup hash mismatch')
result={'image':str(image),'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
        'mode':args.mode,'target':'mimxrt1064','frequency':args.frequency,'connect':args.connect,'backup_complete':False,
        'program_complete':False,'readback_verified':False,'core_left_halted':False}
if args.reuse_backup:
    result.update(backup_complete=True,backup=str(backup),backup_sha256=previous['sha256'])
def save():
    (evidence/'pyocd-flash-result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
session=ConnectHelper.session_with_chosen_probe(
    unique_id=os.environ['SMARTCAR_PROBE_ID'],target_override='mimxrt1064',
    frequency=args.frequency,connect_mode=args.connect,blocking=False,
    options={'resume_on_disconnect':False})
if session is None: raise RuntimeError('Expected SEEKFREE probe not found')
try:
    with session:
        target=session.target
        target.halt()
        # A running application's IRQs must not enter stale handlers while Flash
        # is erased or its DTCM data is used by the flash algorithm. Reset clears
        # any active HardFault left by a prior failed operation.
        target.reset_and_halt(reset_type=target.ResetType.SYSRESETREQ)
        target.write_core_register('primask',1)
        target.write32(0xE000E010,0) # SysTick disabled
        for bank in range(8):
            target.write32(0xE000E180+bank*4,0xffffffff) # NVIC disable
            target.write32(0xE000E280+bank*4,0xffffffff) # NVIC clear pending
        # Same FlexRAM configuration as the repository's NXP download init file.
        target.write32(0x400AC044,0x55AFFA55)
        target.write32(0x400AC040,target.read32(0x400AC040)|(1<<2))
        print('CONNECTED; core halted',flush=True)
        if args.nxp_flm:
            flm=PackFlashAlgo(str(base/'repair/build/diag/MIMXRT1064_QSPI_4KB_SEC.FLM'))
            assert flm.flash_start==0x70000000 and flm.flash_size==0x400000
            algo=flm.get_pyocd_flash_algo(4096,RamRegion(start=0x20000000,length=0x8000))
            region=target.memory_map.get_region_for_address(0x70000000)
            target.memory_map.remove_region(region)
            target.memory_map.add_region(FlashRegion(name='flexspi_nxp_flm',start=flm.flash_start,
                length=flm.flash_size,blocksize=4096,page_size=flm.page_size,
                is_boot_memory=True,algo=algo))
            target.create_flash()
            result['algorithm']='NXP.MIMXRT1064_DFP.19.0.0/MIMXRT1064_QSPI_4KB_SEC.FLM'
            print('NXP_FLM page='+str(flm.page_size)+' RAM='+hex(algo['load_address']),flush=True)
        # Read directly through the AP to avoid serving cached flash contents.
        memory=target.selected_core.ap
        marker=bytes(memory.read_memory_block8(0x70000000,4))
        print('FLASH_HEADER='+marker.hex(),flush=True)
        if marker!=b'FCFB':
            if not args.reuse_backup:
                raise RuntimeError('Unexpected RT1064 flash configuration header; refusing to erase')
            print('RECOVERING flash interface after reset; verified original backup preserved',flush=True)
        if not args.reuse_backup:
            if backup.exists(): raise RuntimeError('Backup already exists; refusing to overwrite original evidence')
            with backup.open('xb') as stream:
                for offset in range(0,0x400000,0x10000):
                    block=bytes(memory.read_memory_block8(0x70000000+offset,0x10000))
                    if len(block)!=0x10000: raise RuntimeError('Incomplete flash backup')
                    stream.write(block)
                    if (offset+0x10000)%0x80000==0:
                        print('BACKUP_BYTES='+str(offset+0x10000),flush=True)
        data=backup.read_bytes()
        if len(data)!=0x400000: raise RuntimeError('Wrong backup size')
        result.update(backup_complete=True,backup=str(backup),backup_sha256=hashlib.sha256(data).hexdigest())
        backup_meta.write_text(json.dumps({'sha256':result['backup_sha256'],'size':len(data)},indent=2),encoding='utf-8')
        save()
        print('BACKUP_COMPLETE '+result['backup_sha256'],flush=True)
        FileProgrammer(session,chip_erase='sector',smart_flash=False).program(str(image))
        result['program_complete']=True;save()
        target.halt()
        expected=IntelHex(str(image));checked=0
        for first,end in expected.segments():
            for address in range(first,end,4096):
                stop=min(address+4096,end)
                actual=bytes(memory.read_memory_block8(address,stop-address))
                want=bytes(expected.tobinarray(start=address,end=stop-1))
                if actual!=want: raise RuntimeError('Readback mismatch at '+hex(address))
                checked+=len(actual)
        result.update(readback_verified=True,verified_bytes=checked,core_left_halted=True)
        save()
        print('FLASH_READBACK_VERIFIED bytes='+str(checked)+'; CORE_HALTED; battery cold boot required',flush=True)
except Exception as exc:
    result['error']=str(exc);save();raise
