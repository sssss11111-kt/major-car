param([string]$Fixture)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'hold_protocol.ps1')
function Check($value,[string]$label){if(-not $value){throw "FAIL hold protocol: $label"}}
function Reject([scriptblock]$action,[string]$label){$rejected=$false;try{& $action | Out-Null}catch{$rejected=$true};Check $rejected $label}
$lines=Get-Content -LiteralPath $Fixture
$cfgLine=@($lines | Where-Object {$_ -like 'HOLD-CFG *'})[0]
$c=Parse-HoldConfig $cfgLine;Check ($c.cap -eq 3000 -and $c.selected -eq 1800) 'real C configuration'
Reject {Parse-HoldConfig ($cfgLine.Replace('20261019H1','20261019C3'))} 'reject old firmware'
Reject {Parse-HoldConfig ($cfgLine.Replace('guard=1500','guard=0'))} 'reject guard mismatch'
$states=@($lines | Where-Object {$_ -like 'HOLD-STATE *'} | ForEach-Object {Parse-HoldState $_})
$done=@($states | Where-Object {$_.result -eq 1})[-1]
Check ($done.n -eq 200 -and $done.sum -eq 4000 -and $done.hold_ms -eq 3000) 'real full-window statistics'
Check ($done.kick_ticks -eq 3) 'sticky confirmed kick samples'
Check ((Get-HoldOutcome $done $done.id 1 1800) -eq 'continuous') 'continuous result'
$doneLine=@($lines | Where-Object {$_ -like 'HOLD-STATE *' -and $_ -match 'result=1 '})[-1]
$inter=Parse-HoldState ($doneLine.Replace('min=20','min=0').Replace('zeros=0','zeros=1'))
Check ((Get-HoldOutcome $inter $inter.id 1 1800) -eq 'intermittent') 'intermittent feedback is not continuous'
Reject {Get-HoldOutcome $done ($done.id+1) 1 1800} 'old trial identity'
Reject {Get-HoldOutcome $done $done.id 2 1800} 'wrong side'
$manual=Parse-HoldState ($doneLine.Replace('phase=3 result=1','phase=0 result=0'))
Reject {Get-HoldOutcome $manual $manual.id 1 1800} 'manual stop is not completion'
$bad=Parse-HoldState ($doneLine.Replace('fault=0','fault=5'))
Reject {Get-HoldOutcome $bad $bad.id 1 1800} 'fault is not completion'
$badKick=Parse-HoldState ($doneLine.Replace('kick_ticks=3','kick_ticks=2'))
Reject {Get-HoldOutcome $badKick $badKick.id 1 1800} 'unconfirmed kick cannot pass'
$lost=Parse-HoldState ($doneLine.Replace('result=1','result=2').Replace('total_ms=3200','total_ms=700').Replace('hold_ms=3000','hold_ms=500').Replace('n=200 sum=4000 min=20 max=20 zeros=0','n=0 sum=0 min=0 max=0 zeros=0'))
Check ((Get-HoldOutcome $lost $lost.id 1 1800) -eq 'lost_motion') 'lost motion is an expected failed measurement'
Write-Output 'Hold protocol: real C fixture / version / outcome / interruption / identity PASS'
