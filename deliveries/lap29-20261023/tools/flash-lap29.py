"""Direct LAP29 sector programming; no read/backup of the old application."""
from pathlib import Path
import argparse, hashlib, json, logging, sys
from datetime import datetime, timezone
from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper
from pyocd.probe.aggregator import DebugProbeAggregator
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.target.pack.flash_algo import PackFlashAlgo
from pyocd.core.memory_map import RamRegion

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--project', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--prepared', action='store_true')
    args = ap.parse_args()
    if not args.prepared:
        raise SystemExit('Motor power OFF, logic power ON, debugger connected must be confirmed.')
    root = args.project.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    result = {'version':'C11LAP29', 'started_utc':datetime.now(timezone.utc).isoformat(),
              'success':False, 'old_application_read_or_backed_up':False,
              'prepared_confirmed':True, 'programming_started':False}
    try:
        image = root/'demo/mdk/Objects/rt1064.hex'
        digest = hashlib.sha256(image.read_bytes()).hexdigest()
        assert digest == '3dbfd5966a0baf6befb10d373b2b1b501097aa5a21e15997c45b93114f44b5e7', digest
        status = json.loads((root/'verification/LAP29-status.json').read_text(encoding='utf-8-sig'))
        assert status['built'] and status['build_errors']==0 and status['build_warnings']==0
        h = IntelHex(str(image)); segments = h.segments()
        assert segments == [(1879048192, 1879121920)], segments
        result.update(hex_sha256=digest, segments=segments, image_bytes=73728)
        probes = DebugProbeAggregator.get_all_connected_probes()
        assert len(probes)==1, 'Exactly one debugger required'
        s = ConnectHelper.session_with_chosen_probe(unique_id=probes[0].unique_id,
            target_override='mimxrt1064', frequency=1000000,
            options={'connect_mode':'halt','resume_on_disconnect':False,
                     'smart_flash':False,'keep_unwritten':False})
        assert s is not None
        flm_path = root/'verification/MIMXRT1064_QSPI_4KB_SEC.FLM'
        flm = PackFlashAlgo(str(flm_path))
        assert flm.flash_start==0x70000000 and flm.flash_size==0x400000
        region = s.target.memory_map.get_region_for_address(0x70000000)
        region.algo = flm.get_pyocd_flash_algo(256, RamRegion(start=0x20200000,length=0x10000))
        with s:
            t=s.target; t.halt()
            result['programming_started']=True
            print('PROGRAMMING C11LAP29: sector erase; smart comparison and preservation disabled',flush=True)
            FileProgrammer(s,chip_erase='sector',smart_flash=False,keep_unwritten=False).program(str(image))
            t.reset_and_halt()
            total=0; readback_hash=hashlib.sha256()
            for a,b in segments:
                for start in range(a,b,4096):
                    end=min(start+4096,b)
                    actual=bytes(t.read_memory_block8(start,end-start))
                    assert actual==h.tobinarray(start=start,end=end-1).tobytes(), hex(start)
                    total+=end-start; readback_hash.update(actual)
            assert t.get_state().name=='HALTED'
            result.update(success=True,readback_match=True,verified_bytes=total,
                          programmed_data_sha256=readback_hash.hexdigest(),state='HALTED',
                          cold_boot_required=True,hardware_track_tested=False)
    except Exception as e:
        result['error']=str(e)
        raise
    finally:
        result['finished_utc']=datetime.now(timezone.utc).isoformat()
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(result),flush=True)

if __name__=='__main__':
    logging.basicConfig(level=logging.WARNING)
    main()
