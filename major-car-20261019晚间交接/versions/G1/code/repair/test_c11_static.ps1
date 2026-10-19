param([ValidateSet('query','high','low')][string]$Action='query')
$ErrorActionPreference='Stop'
$device=Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq 'COM6'
if(-not $device -or $device.PNPDeviceID -notmatch 'VID_2E3C&PID_5740'){throw 'Wireless USB identity mismatch'}
$folder=Join-Path $PSScriptRoot ('evidence\c11-'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$port=[IO.Ports.SerialPort]::new('COM6',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$port.DtrEnable=$false;$port.RtsEnable=$false;$port.WriteTimeout=500
$capture=[IO.MemoryStream]::new();$commands=[Collections.Generic.List[string]]::new();$failure=$null
function Receive-For([int]$ms){
    $timer=[Diagnostics.Stopwatch]::StartNew();$buffer=New-Object byte[] 4096
    while($timer.ElapsedMilliseconds -lt $ms){
        if($port.BytesToRead){$n=$port.Read($buffer,0,[Math]::Min($buffer.Length,$port.BytesToRead));$capture.Write($buffer,0,$n)}
        else{Start-Sleep -Milliseconds 10}
    }
}
function Send-Query([string]$command){$port.Write($command);$commands.Add($command)}
try{
    $port.Open();$port.DiscardInBuffer()
    Send-Query 'p';Send-Query 'q';Receive-For 2000
    $text=[Text.Encoding]::ASCII.GetString($capture.ToArray())
    if($text -notmatch 'UART8-DIAG[^\r\n]*diag=1 ab=1 ' -or
       $text -notmatch 'C11-STATIC v=20261017E cfg=[01] pad=[01] motors=disabled' -or
       $text -notmatch 'mode=0 fault=0[^\r\n]*pwm=0,0 '){throw 'Static diagnostic preflight missing; no level change sent'}
    if($Action -ne 'query'){
        Send-Query $(if($Action -eq 'high'){'u'}else{'v'})
        Receive-For 300;Send-Query 'q';Receive-For 1300
        $text=[Text.Encoding]::ASCII.GetString($capture.ToArray())
        $wanted=if($Action -eq 'high'){'1'}else{'0'}
        $matches=[regex]::Matches($text,'C11-STATIC v=20261017E cfg=([01]) pad=([01]) motors=disabled')
        if(-not $matches.Count -or $matches[-1].Groups[1].Value -ne $wanted){throw 'Requested C11 output not acknowledged'}
    }
}catch{$failure=$_.Exception.Message}
finally{
    if($port.IsOpen){$port.Close()};$port.Dispose()
    $raw=$capture.ToArray();$capture.Dispose();[IO.File]::WriteAllBytes((Join-Path $folder 'serial.bin'),$raw)
    $text=[Text.Encoding]::ASCII.GetString($raw)
    $result=[ordered]@{action=$Action;failure=$failure;commands=@($commands.ToArray());
      c11=@([regex]::Matches($text,'C11-STATIC[^\r\n]*') | Select-Object -Last 3 | ForEach-Object Value);
      registers=@([regex]::Matches($text,'C7-REG[^\r\n]*') | Select-Object -Last 1 | ForEach-Object Value);
      status=@([regex]::Matches($text,'ms=\d+ mode=\d+[^\r\n]*') | Select-Object -Last 1 | ForEach-Object Value);evidence=$folder}
    $json=$result | ConvertTo-Json -Depth 4
    [IO.File]::WriteAllText((Join-Path $folder 'result.json'),$json,[Text.UTF8Encoding]::new($false));$json
}
if($failure){exit 1}
