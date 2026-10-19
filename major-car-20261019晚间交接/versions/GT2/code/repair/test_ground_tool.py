import unittest,tempfile,json
from pathlib import Path
from unittest.mock import patch
import ground_trial
from ground_trial import parse_state,validate_config,check_state,classify,performance
CFG='GROUND-CFG v=20261019GL2 selected=0 min=0 max=0 step=0 settle=200 kick=3000 kick_min=200 kick_max=300 hold=1500 total=2000 lease=500 guard=120 us=1500 target=20 cap=1500 kp10=40 ki10=200 ff=1206,1198 transfer=200 entry=ff simulated=0'
LINE='GROUND-STATE ms=2000 mode=0 fault=0 id=1 phase=3 done=1 elapsed=1900 duty=0 enc=0,0 pwm=0,0 us=1500 n=50,50 sum=1000,1000 min=20,20 max=20,20 e2=0,0 regulating=3 target=20'
class Contracts(unittest.TestCase):
 def test_config(self):
  self.assertEqual(validate_config(CFG)['target'],20)
  for a,b in [('20261019GL2','20261019GH2'),('cap=1500','cap=2000'),('transfer=200','transfer=1000'),('ff=1206,1198','ff=700,800'),('simulated=0','simulated=1'),('entry=ff','entry=cap')]:
   with self.assertRaises(RuntimeError):validate_config(CFG.replace(a,b))
 def test_completion(self):
  self.assertEqual(classify(parse_state(LINE),1,0),'completed')
  with self.assertRaises(RuntimeError):classify(parse_state(LINE.replace('id=1','id=2')),1,0)
 def test_zero_or_untransferred_not_passed(self):
  for a,b in [('min=20,20','min=0,20'),('regulating=3','regulating=0')]:
   with self.assertRaises(RuntimeError):classify(parse_state(LINE.replace(a,b)),1,0)
 def test_performance_is_separate(self):
  self.assertTrue(performance(parse_state(LINE))['passed'])
  self.assertFalse(performance(parse_state(LINE.replace('sum=1000,1000','sum=1500,1000')))['passed'])
 def test_guards(self):
  for a,b in [('fault=0','fault=5'),('enc=0,0','enc=121,0'),('us=1500','us=1490')]:
   with self.assertRaises(RuntimeError):check_state(parse_state(LINE.replace(a,b)))
  with self.assertRaises(RuntimeError):check_state(parse_state(LINE.replace('mode=0','mode=10').replace('phase=3','phase=2').replace('pwm=0,0','pwm=1600,1400')))
 def test_fixture(self):
  text=(Path(__file__).parents[1]/'competition/firmware/camera_car/tests/out/ground-fixture.txt').read_text(encoding='ascii')
  configs=[validate_config(x) for x in text.splitlines() if x.startswith('GROUND-CFG ')]
  self.assertTrue(configs)
  done=[parse_state(x) for x in text.splitlines() if x.startswith('GROUND-STATE ') and 'done=1' in x]
  self.assertTrue(done);self.assertEqual(classify(done[-1],1,0),'completed');self.assertTrue(performance(done[-1])['passed'])
 def exercise(self,run=False,fault=False,silent_stop=False):
  class Port:
   def __init__(self):self.now=0.;self.start=None;self.stopped=False;self.pending=bytearray();self.commands=''
   @property
   def in_waiting(self):return len(self.pending)
   def reset_input_buffer(self):self.pending.clear()
   def write(self,data):
    for c in data.decode():
     self.commands+=c
     if c=='Y':self.start=self.now
     if c=='s':self.stopped=True
     if c=='W':
      if silent_stop and self.stopped:continue
      elapsed=int((self.now-self.start)*1000) if self.start is not None else 0
      active=self.start is not None and not self.stopped and elapsed<1900
      done=self.start is not None and not self.stopped and not active
      line=LINE.replace('ms=2000','ms='+str(int(self.now*1000))).replace('id=1','id='+str(int(self.start is not None)))
      if active:
       line=line.replace('mode=0','mode=10').replace('phase=3','phase=2').replace('done=1','done=0').replace('elapsed=1900','elapsed='+str(elapsed)).replace('enc=0,0','enc=20,20').replace('pwm=0,0','pwm=1200,1200')
       if fault:line=line.replace('fault=0','fault=5').replace('mode=10','mode=3')
      elif not done:line=line.replace('done=1','done=0')
      self.pending.extend((line+'\r\n').encode())
      if not active:self.pending.extend((CFG+'\r\n').encode())
   def read(self,n):
    self.now+=.025;chunk=bytes(self.pending[:n]);del self.pending[:n];return chunk
  port=Port()
  with tempfile.TemporaryDirectory() as d,patch.object(ground_trial.time,'monotonic',lambda:port.now):
   out=Path(d)/'trial'
   if fault or silent_stop:
    with self.assertRaises(RuntimeError):ground_trial.session(port,out,run=run)
   else:ground_trial.session(port,out,run=run)
   r=json.loads((out/'result.json').read_text(encoding='utf-8'))
   self.assertEqual(port.commands.count('Y'),int(run))
   if run:self.assertEqual(r['final_zero_verified'],not silent_stop)
   self.assertFalse('<' in port.commands or '>' in port.commands)
   return r
 def test_default_only_queries(self):self.assertFalse(self.exercise()['motion_sent'])
 def test_single_run(self):self.assertTrue(self.exercise(run=True)['performance']['passed'])
 def test_fault_no_retry(self):self.assertEqual(self.exercise(run=True,fault=True)['outcome'],'error')
 def test_final_zero_missing_fails(self):self.assertEqual(self.exercise(run=True,silent_stop=True)['outcome'],'error')
if __name__=='__main__':unittest.main()
