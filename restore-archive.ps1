$ErrorActionPreference = 'Stop'
$archiveName = -join ([char[]](50,48,50,54,49,48,50,49,19978,21320,20363,31243,27979,35797))
$outPath = Join-Path $PSScriptRoot ($archiveName + '.zip')
if (Test-Path -LiteralPath $outPath) { throw 'Output already exists; move it before rebuilding.' }
$parts = @(Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'archive') -Filter '*.part*' | Sort-Object Name)
if ($parts.Count -ne 22) { throw 'Expected exactly 22 parts.' }
$dest = [IO.File]::Open($outPath, [IO.FileMode]::CreateNew)
try {
 foreach ($part in $parts) {
  $src = [IO.File]::OpenRead($part.FullName)
  try { $src.CopyTo($dest) } finally { $src.Dispose() }
 }
} finally { $dest.Dispose() }
$hasher = [Security.Cryptography.SHA256]::Create()
$inputFile = [IO.File]::OpenRead($outPath)
try { $actual = [BitConverter]::ToString($hasher.ComputeHash($inputFile)).Replace('-', '').ToLowerInvariant() }
finally { $inputFile.Dispose(); $hasher.Dispose() }
if ($actual -ne '5d0265ff6ed707da3c6f862e9dbb5fc1ce84e34da0ef5b5e7e77ebd3fac3637f') { throw 'SHA256 mismatch.' }
Write-Output ('Verified: ' + $outPath)
