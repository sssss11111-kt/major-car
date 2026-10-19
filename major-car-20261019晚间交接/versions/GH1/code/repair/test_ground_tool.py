import unittest, tempfile
from unittest.mock import patch
import ground_trial
from pathlib import Path
from ground_trial import parse_state,validate_config,check_state,classify
CFG='GROUND-CFG v=20261019GH1 selected=1000 min=900 max=1500 step=100 settle=200 kick=3000 kick_min=200 kick_max=300 hold=1500 total=2000 lease=500 guard=120 us=1500 simulated=0'
LINE='GROUND-STATE ms=2000 mode=0 fault=0 id=1 phase=3 done=1 elapsed=1900 duty=1000 enc=0,0 pwm=0,0 us=1500 n=50,50 sum=1000,1000 min=20,20 max=20,20'
class GroundContracts(unittest.TestCase):
 def test_config(self):
  self.assertEqual(validate_config(CFG)['selected'],1000)
  for a,b in [('20261019GH1','20261019G1'),('total=2000','total=6000'),('guard=120','guard=1500'),('simulated=0','simulated=1'),('selected=1000','selected=7000')]:
   with self.assertRaises(RuntimeError):validate_config(CFG.replace(a,b))
 def test_state_and_completion(self):
  s=parse_state(LINE);check_state(s);self.assertEqual(classify(s,1,1000),'completed')
  for a,b in [('id=1','id=2'),('duty=1000','duty=1100'),('elapsed=1900','elapsed=500'),('min=20,20','min=0,20'),('n=50,50','n=0,0')]:
   with self.assertRaises(RuntimeError):classify(parse_state(LINE.replace(a,b)),1,1000)
 def test_fault_and_wrong_steering(self):
  for a,b in [('fault=0','fault=5'),('enc=0,0','enc=-6,0'),('pwm=0,0','pwm=3001,0'),('us=1500','us=1510')]:
   with self.assertRaises(RuntimeError):check_state(parse_state(LINE.replace(a,b)))
 def test_actual_c_fixture(self):
  text=(Path(__file__).parents[1]/'competition/firmware/camera_car/tests/out/ground-fixture.txt').read_text(encoding='ascii')
  configs=[validate_config(s) for s in text.splitlines() if s.startswith('GROUND-CFG ')]
  self.assertTrue(configs)
  complete=[parse_state(s) for s in text.splitlines() if s.startswith('GROUND-STATE ') and 'done=1' in s]
  self.assertTrue(complete);self.assertEqual(classify(complete[-1],1,1000),'completed')
 def test_truncated(self):
  self.assertIsNone(parse_state('GROUND-STATE ms=10 pwm=0,'))

 def exercise(self,run=False,fault=False,silent_stop=False):
  class Port:
   def __init__(self):self.now=0.;self.start=None;self.stopped=False;self.pending=bytearray();self.selected=1000;self.commands=''
   @property
   def in_waiting(self):return len(self.pending)
   def reset_input_buffer(self):self.pending.clear()
   def write(self,data):
    for c in data.decode():
     self.commands+=c
     if c=='<':self.selected=max(900,self.selected-100)
     if c=='>':self.selected=min(1500,self.selected+100)
     if c=='Y':self.start=self.now
     if c=='s':self.stopped=True
     if c=='W':
      if silent_stop and self.stopped:continue
      elapsed=int((self.now-self.start)*1000) if self.start is not None else 0
      active=self.start is not None and not self.stopped and elapsed<1900
      done=self.start is not None and not self.stopped and not active
      line=LINE.replace('ms=2000','ms='+str(int(self.now*1000))).replace('id=1','id='+str(int(self.start is not None))).replace('duty=1000','duty='+str(self.selected))
      if active:
       line=line.replace('mode=0','mode=10').replace('phase=3','phase=2').replace('done=1','done=0').replace('elapsed=1900','elapsed='+str(elapsed)).replace('enc=0,0','enc=20,20').replace('pwm=0,0','pwm=1000,1000')
       if fault:line=line.replace('fault=0','fault=5').replace('mode=10','mode=3')
      elif not done:line=line.replace('done=1','done=0')
      self.pending.extend((line+'\r\n').encode())
      if not active:self.pending.extend((CFG.replace('selected=1000','selected='+str(self.selected))+'\r\n').encode())
   def read(self,n):
    self.now+=.025
    chunk=bytes(self.pending[:n]);del self.pending[:n];return chunk
  port=Port()
  with tempfile.TemporaryDirectory() as d,patch.object(ground_trial.time,'monotonic',lambda:port.now):
   out=Path(d)/'trial'
   if fault or silent_stop:
    with self.assertRaises(RuntimeError):ground_trial.session(port,out,run=run)
   else:ground_trial.session(port,out,run=run)
   import json
   result=json.loads((out/'result.json').read_text(encoding='utf-8'))
   self.assertEqual(port.commands.count('Y'),int(run))
   if run:self.assertEqual(result['final_zero_verified'],not silent_stop)
   return result
 def test_missing_stop_feedback_fails(self):self.assertEqual(self.exercise(run=True,silent_stop=True)['outcome'],'error')
 def test_default_never_starts(self):self.assertFalse(self.exercise()['motion_sent'])
 def test_one_start_normal_expiry(self):self.assertEqual(self.exercise(run=True)['outcome'],'completed')
 def test_fault_stops_without_retry(self):self.assertEqual(self.exercise(run=True,fault=True)['outcome'],'error')

if __name__=='__main__':unittest.main()
