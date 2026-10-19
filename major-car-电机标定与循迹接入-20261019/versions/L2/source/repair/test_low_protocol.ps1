param([string]$Fixture)
$ErrorActionPreference='Stop';. (Join-Path $PSScriptRoot 'low_protocol.ps1')
function Check($condition){if(-not $condition){throw 'LOW protocol assertion'}}
function Reject([scriptblock]$fn){$failed=$false;try{& $fn|Out-Null}catch{$failed=$true};Check $failed}
$text=Get-Content -Raw -LiteralPath $Fixture
$configs=[regex]::Matches($text,'LOW-CFG [^\r\n]*\r\n');Check ($configs.Count -ge 3)
$cfg=ParseLowConfig $configs[0].Value;Check ($cfg.selected -eq 20)
Reject {ParseLowConfig ($configs[0].Value.Replace('guard=450','guard=1500'))}
Reject {ParseLowConfig ($configs[0].Value.Replace('20261019L2','20261019H1'))}
$done=@(foreach($m in [regex]::Matches($text,'LOW-STATE [^\r\n]*\r\n')){$s=ParseLowState $m.Value;if($s.done -eq 1){$s}})
foreach($motor in @(1,2,3)){
 $s=@($done|Where-Object motor -eq $motor)[-1];Check ($null -ne $s)
 Check ((LowOutcome $s $s.id $motor 20) -eq 'performance_pass')
 Reject {LowOutcome $s ($s.id+1) $motor 20}
 $bad=ParseLowState (([regex]::Matches($text,'LOW-STATE [^\r\n]*\r\n')|Where-Object Value -Match "done=1 elapsed=6000 motor=$motor ")[-1].Value)
 $bad.done=0;Reject {LowOutcome $bad $bad.id $motor 20}
}
$s=$done[-1];$s.e2[0]=30000;Check ((LowOutcome $s $s.id 3 20) -eq 'needs_tuning')
$s.fault=5;Reject {LowOutcome $s $s.id 3 20}
'LOW actual-C-fixture protocol / profile / identity / incomplete / tuning / fault PASS'
