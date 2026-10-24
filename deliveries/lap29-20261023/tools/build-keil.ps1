param([string]$Keil='D:\Keil5\UV4\UV4.exe')
$ErrorActionPreference='Stop'
$delivery=Split-Path -Parent $PSScriptRoot
$project=Join-Path $delivery 'project\demo\mdk\rt1064.uvprojx'
$log=Join-Path $delivery 'project\verification\build-reproduced.log'
if(-not (Test-Path -LiteralPath $Keil)){throw 'Set -Keil to your licensed UV4.exe path'}
$proc=Start-Process -FilePath $Keil -ArgumentList @('-b',('"'+$project+'"'),'-j0','-o',('"'+$log+'"')) -WindowStyle Hidden -PassThru
$proc.WaitForExit()
Get-Content -LiteralPath $log -Tail 10
if(-not ((Get-Content -LiteralPath $log -Raw).Contains('0 Error(s), 0 Warning(s)'))){throw 'Build failed; inspect the log'}
Write-Output 'Build passed. This does not flash or prove the newly generated HEX matches the archived burned HEX.'
