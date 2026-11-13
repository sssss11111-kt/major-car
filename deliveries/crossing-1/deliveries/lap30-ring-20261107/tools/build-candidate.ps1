param([string]$Keil='D:\Keil5\UV4\UV4.exe')
$ErrorActionPreference='Stop'
$ringProject=Join-Path (Split-Path -Parent $PSScriptRoot) 'project'
$ringLog=Join-Path $ringProject 'verification\build-LAP30.log'
$ringStarted=Get-Date
if(Test-Path -LiteralPath 'R:\'){throw 'R: is already in use'}
$ringRoot=[IO.Path]::GetFullPath((Join-Path $ringProject '..\..\..'))
subst R: $ringRoot
try {
 $ringMapped='R:\deliveries\lap30-ring-20261107\project'
 $ringProc=Start-Process -FilePath $Keil -ArgumentList @('-r',"$ringMapped\demo\mdk\rt1064.uvprojx",'-j0','-o',"$ringMapped\verification\build-LAP30.log") -WorkingDirectory "$ringMapped\demo\mdk" -WindowStyle Hidden -PassThru
 $ringProc.WaitForExit()
 if(-not (Test-Path -LiteralPath $ringLog)){throw 'Missing build log'}
 if((Get-Item -LiteralPath $ringLog).LastWriteTime -lt $ringStarted.AddSeconds(-2)){throw 'Stale build log'}
 Get-Content -LiteralPath $ringLog -Tail 10
 if($ringProc.ExitCode -ne 0){throw "Build exit $($ringProc.ExitCode)"}
 if(-not ((Get-Content -LiteralPath $ringLog -Raw).Contains('0 Error(s), 0 Warning(s)'))){throw 'Build failed'}
 foreach($ringOutput in @('rt1064.axf','rt1064.hex')) {
  if((Get-Item -LiteralPath (Join-Path $ringProject "demo\mdk\Objects\$ringOutput")).LastWriteTime -lt $ringStarted.AddSeconds(-2)){throw 'Stale firmware'}
 }
 Write-Output 'Fresh LAP30 build verified; no hardware accessed.'
} finally {subst R: /D}
