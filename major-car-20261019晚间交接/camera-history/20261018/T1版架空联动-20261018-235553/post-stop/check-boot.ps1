$ErrorActionPreference='Stop'
$p=[IO.Ports.SerialPort]::new('COM6',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$p.DtrEnable=$false;$p.RtsEnable=$false;$p.WriteTimeout=500;$p.ReadTimeout=100
$all=[IO.MemoryStream]::new();$buffer=[byte[]]::new(4096)
function Receive([int]$ms){$timer=[Diagnostics.Stopwatch]::StartNew();while($timer.ElapsedMilliseconds -lt $ms){$n=$p.BytesToRead;if($n){$k=$p.Read($buffer,0,[Math]::Min($n,$buffer.Length));$all.Write($buffer,0,$k)}else{Start-Sleep -Milliseconds 10}}}
try{$p.Open();Receive 1200;$p.Write('dpq');Receive 1600;$p.Write('p');Receive 1000
 $bytes=$all.ToArray();[IO.File]::WriteAllBytes((Join-Path $PSScriptRoot 'serial.bin'),$bytes);$text=[Text.Encoding]::ASCII.GetString($bytes);[IO.File]::WriteAllText((Join-Path $PSScriptRoot 'serial.txt'),$text);Write-Output $text
}finally{if($p.IsOpen){$p.Close()};$p.Dispose();$all.Dispose()}
