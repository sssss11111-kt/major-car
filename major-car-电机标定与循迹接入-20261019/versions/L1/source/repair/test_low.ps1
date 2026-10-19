param([ValidateSet(1,2,3)][int]$Motor=1,[ValidateSet(10,15,20,25,30,35,40)][int]$Target=20,[string]$PortName='COM9',[switch]$Run)
$ErrorActionPreference='Stop';. (Join-Path $PSScriptRoot 'low_protocol.ps1')
$dev=Get-CimInstance Win32_SerialPort|Where-Object DeviceID -eq $PortName
if(-not $dev -or $dev.PNPDeviceID -notmatch 'VID_2E3C&PID_5740'){throw 'Wrong wireless USB identity'}
$stamp=[TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([DateTime]::UtcNow,'China Standard Time').ToString('yyyyMMdd-HHmmss')
$dir=Join-Path $PSScriptRoot ('evidence/low-'+$stamp);New-Item -ItemType Directory -Path $dir -Force|Out-Null
$port=[IO.Ports.SerialPort]::new($PortName,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$port.DtrEnable=$false;$port.RtsEnable=$false;$port.WriteTimeout=300
$raw=[IO.MemoryStream]::new();$commands=[Collections.Generic.List[object]]::new()
$failure=$null;$outcome='preflight_only';$completion=$null;$cfg=$null;$final=$null;$pre=$null;$motionSent=$false;$finalOffset=0
function Send([string]$c){$port.Write($c);$commands.Add(@{utc=[DateTime]::UtcNow.ToString('o');command=$c})}
function Receive([int]$ms){$watch=[Diagnostics.Stopwatch]::StartNew();$b=New-Object byte[] 8192;while($watch.ElapsedMilliseconds -lt $ms){if($port.BytesToRead){$n=$port.Read($b,0,[Math]::Min($b.Length,$port.BytesToRead));$raw.Write($b,0,$n)}else{Start-Sleep -Milliseconds 5}}}
function Text([int]$offset=0){$b=$raw.ToArray();return [Text.Encoding]::ASCII.GetString($b,$offset,$b.Length-$offset)}
function States([string]$text){foreach($m in [regex]::Matches($text,'LOW-STATE [^\r\n]*\r\n')){ParseLowState $m.Value}}
function Idle($s){if(-not $s -or $s.mode -ne 0 -or $s.fault -ne 0 -or $s.pwm[0] -ne 0 -or $s.pwm[1] -ne 0 -or $s.enc[0] -ne 0 -or $s.enc[1] -ne 0){throw 'Require idle zero output/feedback'}}
function QueryIdle {
 # H1 evidence showed an isolated missing config packet. Repeat queries only,
 # never retry a motion command. Reject malformed or changed profiles immediately.
 $offset=[int]$raw.Length
 for($i=0;$i -lt 3;$i++){
  Send 'Q';Receive 650;$t=Text $offset;$matchesCfg=[regex]::Matches($t,'LOW-CFG [^\r\n]*\r\n');$ss=@(States $t)
  if($matchesCfg.Count -and $ss.Count){$c=ParseLowConfig $matchesCfg[-1].Value;Idle $ss[-1];return @{config=$c;state=$ss[-1]}}
 };throw 'No complete idle L1 config/state'
}
try{
 $port.Open();$port.DiscardInBuffer();$q=QueryIdle;$cfg=$q.config;$pre=$q.state
 Send 'q';Receive 650;$t=Text
 if($t -notmatch 'UART8-DIAG[^\r\n]*diag=0 ab=1 ' -or $t -notmatch 'MOTOR-DIR v=20261017I left=C11 cfg=1 sample=[01] right=C9 cfg=0'){throw 'Wireless/direction profile not confirmed'}
 if($Run){
  Send ('G'*6);Receive 450;if($Target -gt 10){Send ('F'*(($Target-10)/5));Receive 450}
  $q=QueryIdle;$cfg=$q.config;$pre=$q.state;if([int]$cfg.selected -ne $Target){throw 'LOW selected target not acknowledged'}
  $previous=$pre.id;$id=$null;$offset=[int]$raw.Length
  Send $(if($Motor -eq 1){'J'}elseif($Motor -eq 2){'N'}else{'B'});$motionSent=$true
  $watch=[Diagnostics.Stopwatch]::StartNew();$next=0;$lastRx=0;$lastMs=-1
  while($watch.ElapsedMilliseconds -lt 7500){
   if($watch.ElapsedMilliseconds -ge $next){Send 'Q';$next+=100};Receive 20;$t=Text $offset
   if($t.Contains('LOW-START REFUSED')){throw 'LOW start refused'}
   $ss=@(States $t)
   if($ss.Count){$s=$ss[-1];if($s.ms -ne $lastMs){$lastMs=$s.ms;$lastRx=$watch.ElapsedMilliseconds}
    if($s.id -ne $previous){if($null -eq $id){$id=$s.id;if($id -ne (($previous+1)%4294967296)){throw 'Unexpected new LOW identity'}}
     $candidate=LowOutcome $s $id $Motor $Target
     if($candidate -ne 'pending'){$completion=$s;$outcome=$candidate;break}
    }
   }
   if($watch.ElapsedMilliseconds-$lastRx -gt 700){throw 'LOW telemetry stale'}
  }
  $ack="LOW-START v=20261019L1 id=$id motor=$Motor target=$Target duration=6000 lease=500"
  if(-not $completion -or -not (Text $offset).Contains($ack)){throw 'Missing LOW identified start/completion'}
 }
}catch{$failure=$_.Exception.Message;$outcome='error'}
finally{
 $finalOffset=[int]$raw.Length
 if($port.IsOpen){try{Send 's';Receive 1000;$finalOffset=[int]$raw.Length;Send 'Q';Receive 650}catch{};$port.Close()};$port.Dispose()
 [IO.File]::WriteAllBytes((Join-Path $dir 'serial.bin'),$raw.ToArray())
 try{$ss=@(States (Text $finalOffset));if($ss.Count){$final=$ss[-1]}}catch{if(-not $failure){$failure=$_.Exception.Message;$outcome='error'}}
 $zero=$final -and $final.mode -eq 0 -and $final.fault -eq 0 -and $final.pwm[0] -eq 0 -and $final.pwm[1] -eq 0 -and $final.enc[0] -eq 0 -and $final.enc[1] -eq 0
 if(-not $failure -and -not $zero){$failure='Final zero output/feedback not confirmed';$outcome='error'}
 $raw.Dispose()
 $r=[ordered]@{version='20261019L1';motor=$Motor;target=$Target;outcome=$outcome;failure=$failure;motion_sent=$motionSent;measurement_complete=($null -ne $completion -and -not $failure);configuration=$cfg;completion=$completion;final=$final;final_zero=$zero;commands=@($commands.ToArray());physical_observation='pending';evidence=$dir}
 $r|ConvertTo-Json -Depth 7|Set-Content (Join-Path $dir 'result.json') -Encoding utf8
 [pscustomobject]@{outcome=$outcome;failure=$failure;completion=$completion;final_zero=$zero;evidence=$dir}|ConvertTo-Json -Depth 5
}
if($failure){exit 1};exit 0
