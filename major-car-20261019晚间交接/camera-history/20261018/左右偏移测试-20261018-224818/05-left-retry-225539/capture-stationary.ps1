param([string]$PortName='COM6')
$ErrorActionPreference='Stop'
$folder=$PSScriptRoot
$port=[System.IO.Ports.SerialPort]::new($PortName,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$port.ReadTimeout=500;$port.WriteTimeout=1000;$port.DtrEnable=$false;$port.RtsEnable=$false
function Read-For([int]$ms){
 $stream=[IO.MemoryStream]::new();$timer=[Diagnostics.Stopwatch]::StartNew();$buffer=[byte[]]::new(8192)
 while($timer.ElapsedMilliseconds -lt $ms){$n=$port.BytesToRead;if($n){$k=$port.Read($buffer,0,[Math]::Min($n,$buffer.Length));$stream.Write($buffer,0,$k)}else{Start-Sleep -Milliseconds 10}}
 $bytes=$stream.ToArray();$stream.Dispose();return ,$bytes
}
try {
 $port.Open();$port.Write('s');Start-Sleep -Milliseconds 150;$port.DiscardInBuffer()
 $port.Write('dp');$status=Read-For 1400
 [IO.File]::WriteAllBytes((Join-Path $folder 'version-status.bin'),$status)
 [Text.Encoding]::ASCII.GetString($status)|Write-Output
 foreach($kind in @('gray','binary','gray-repeat')){
  $port.DiscardInBuffer();$port.Write($(if($kind -eq 'binary'){'b'}else{'g'}))
  $bytes=Read-For 6500
  [IO.File]::WriteAllBytes((Join-Path $folder ($kind+'-serial.bin')),$bytes)
  $found=-1
  for($i=0;$i -le $bytes.Length-4;$i++){if($bytes[$i]-eq 0 -and $bytes[$i+1]-eq 255 -and $bytes[$i+2]-eq 1 -and $bytes[$i+3]-eq 1){$found=$i;break}}
  if($found -lt 0 -or $bytes.Length-$found-4 -lt 22560){throw "Incomplete $kind frame: bytes=$($bytes.Length) header=$found"}
  $pgm=[IO.MemoryStream]::new();$header=[Text.Encoding]::ASCII.GetBytes("P5`n188 120`n255`n");$pgm.Write($header,0,$header.Length);$pgm.Write($bytes,$found+4,22560)
  [IO.File]::WriteAllBytes((Join-Path $folder ($kind+'.pgm')),$pgm.ToArray());$pgm.Dispose()
  Write-Output "CAPTURED $kind 188x120 header_offset=$found total_bytes=$($bytes.Length)"
 }
 $port.Write('p');$final=Read-For 700;[IO.File]::WriteAllBytes((Join-Path $folder 'final-status.bin'),$final);[Text.Encoding]::ASCII.GetString($final)|Write-Output
}finally{if($port.IsOpen){$port.Close()};$port.Dispose()}
