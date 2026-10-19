param(
    [ValidateSet('camera','mock')][string]$Mode='camera',
    [string]$Keil='D:\keil5\UV4\UV4.exe'
)
$ErrorActionPreference='Stop'
$taskRoot=$PSScriptRoot
$project=Join-Path $taskRoot 'demo\mdk\rt1064.uvprojx'
$buildLog=Join-Path $taskRoot ("build-"+$Mode+".log")
if (-not (Test-Path -LiteralPath $Keil)) { throw "Keil not found: $Keil" }
if($Mode -eq 'mock') {
    $project=Join-Path $taskRoot 'demo\mdk\rt1064-mock.uvprojx'
}
# Launch the compiler hidden, wait for exit and verify the actual log summary.
$buildProcess=Start-Process -FilePath $Keil -ArgumentList @('-b',('"'+$project+'"'),'-j0','-o',('"'+$buildLog+'"')) -WindowStyle Hidden -PassThru -Wait
if(-not (Test-Path -LiteralPath $buildLog)) { throw 'Build log was not created.' }
$buildText=Get-Content -Raw -LiteralPath $buildLog
Write-Output ($buildText -split "`r?`n" | Select-Object -Last 7)
if($buildProcess.ExitCode -ne 0 -or $buildText -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    throw "Build failed or has warnings: exit $($buildProcess.ExitCode); inspect $buildLog"
}
