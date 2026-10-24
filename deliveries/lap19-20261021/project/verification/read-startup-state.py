from pathlib import Path
import json
import time
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper

p = Path(__file__).resolve().parent
names = {'trial_ticks','frame_tick','good_frames','frame_count','trial_running','trial_fault',
         'auto_phase','wireless_ready','vision_mode','vision_error','model_good','both_rows',
         'usable_rows','snapshot_requested','used','trial_left_pwm','trial_right_pwm','auto_run',
         'control_center','encoder_data_1','encoder_data_2'}
with (p.parent/'demo/mdk/Objects/rt1064.axf').open('rb') as f:
    symbols = {s.name:(s['st_value'],s['st_size']) for s in
               ELFFile(f).get_section_by_name('.symtab').iter_symbols() if s.name in names}
s = ConnectHelper.session_with_chosen_probe(target_override='mimxrt1064',frequency=100000,
        blocking=False,options={'connect_mode':'attach','resume_on_disconnect':False})
if s is None: raise RuntimeError('No debugger')
reports = []
with s:
    for _ in range(4):
        r = {'core_state':str(s.target.get_state())}
        # One contiguous debug memory read; no halt, reset or start command.
        low = min(a for a,n in symbols.values())
        high = max(a+n for a,n in symbols.values())
        raw = bytes(s.target.read_memory_block8(low,high-low))
        for name,(a,n) in symbols.items():
            r[name] = int.from_bytes(raw[a-low:a-low+n], 'little',
                signed=name in {'vision_error','encoder_data_1','encoder_data_2','trial_left_pwm','trial_right_pwm'})
        reports.append(r)
        print(json.dumps(r),flush=True)
        time.sleep(.3)
(p/'startup-state-live.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
