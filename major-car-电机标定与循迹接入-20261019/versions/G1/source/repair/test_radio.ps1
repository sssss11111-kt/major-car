param([string]$PortName='COM6',[int]$BaudRate=115200,[switch]$ServoRegisters)
$ErrorActionPreference='Stop'
$device=Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq $PortName
if(-not $device -or $device.PNPDeviceID -notmatch 'VID_2E3C&PID_5740') {
    throw 'Selected port does not match the identified wireless USB module.'
}
$now=[TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([DateTime]::UtcNow,'China Standard Time')
$folder=Join-Path $PSScriptRoot ('evidence\wireless-'+$now.ToString('yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$port=New-Object System.IO.Ports.SerialPort
$port.PortName=$PortName
$port.BaudRate=$BaudRate
$port.DataBits=8
$port.Parity=[System.IO.Ports.Parity]::None
$port.StopBits=[System.IO.Ports.StopBits]::One
$port.Handshake=[System.IO.Ports.Handshake]::None
$port.DtrEnable=$false
$port.RtsEnable=$false
$port.ReadTimeout=100
$port.WriteTimeout=1000
$records=New-Object 'System.Collections.Generic.List[object]'
function Receive-Phase([string]$Name,[int]$DurationMs) {
    $stream=New-Object System.IO.MemoryStream
    $timer=[System.Diagnostics.Stopwatch]::StartNew()
    $buffer=New-Object byte[] 8192
    try {
        while($timer.ElapsedMilliseconds -lt $DurationMs) {
            $available=$port.BytesToRead
            if($available -gt 0) {
                $count=$port.Read($buffer,0,[Math]::Min($buffer.Length,$available))
                if($count -gt 0){ $stream.Write($buffer,0,$count) }
            } else { Start-Sleep -Milliseconds 20 }
            if($stream.Length -ge 2097152){ break }
        }
        $bytes=$stream.ToArray()
        [IO.File]::WriteAllBytes((Join-Path $folder ($Name+'.bin')),$bytes)
        $ascii=[Text.Encoding]::ASCII.GetString($bytes)
        $status=[regex]::Matches($ascii,'ms=\d+ mode=\d+ fault=\d+ frames=\d+[^\r\n]*')
        $zeros=0
        foreach($b in $bytes){ if($b -eq 0){ $zeros++ } }
        $hex=if($bytes.Length){ [BitConverter]::ToString($bytes,0,[Math]::Min(64,$bytes.Length)) } else { '' }
        $row=[ordered]@{
            phase=$Name; duration_ms=$timer.ElapsedMilliseconds; bytes=$bytes.Length
            zero_bytes=$zeros; nonzero_bytes=($bytes.Length-$zeros)
            status_lines=$status.Count; first_64_hex=$hex
            last_status_lines=@($status | Select-Object -Last 4 | ForEach-Object Value)
        }
        $records.Add($row)
    } finally { $stream.Dispose() }
}
try {
    $port.Open()
    $port.DiscardInBuffer()
    Receive-Phase 'passive' 5000
    for($i=1;$i -le 4;$i++) {
        # Read-only p/q queries, or p/z for servo registers. No motion commands.
        $query=if($i%2 -eq 1){ [byte[]]@(0x70) } elseif($ServoRegisters){ [byte[]]@(0x7a) } else { [byte[]]@(0x71) }
        $port.Write($query,0,1)
        Receive-Phase ('query-'+$i) 2000
    }
    $summary=[ordered]@{
        time_china=$now.ToString('yyyy-MM-dd HH:mm:ss'); port=$PortName
        baud=$BaudRate; framing='8N1'; handshake='None'; dtr=$false; rts=$false
        device_id=$device.PNPDeviceID; commands_sent=$(if($ServoRegisters){@('p','z','p','z')}else{@('p','q','p','q')})
        phases=@($records.ToArray()); evidence_directory=$folder
    }
    $json=$summary | ConvertTo-Json -Depth 8
    [IO.File]::WriteAllText((Join-Path $folder 'result.json'),$json,[Text.UTF8Encoding]::new($false))
    $all=New-Object System.IO.MemoryStream
    foreach($record in $records){
        $raw=[IO.File]::ReadAllBytes((Join-Path $folder ($record.phase+'.bin')))
        $all.Write($raw,0,$raw.Length)
    }
    $text=[Text.Encoding]::ASCII.GetString($all.ToArray())
    $all.Dispose()
    $diagnostics=[regex]::Matches($text,'UART8-DIAG v=20261017[^\r\n]*\r\n')
    $proof=[ordered]@{
        new_firmware_diagnostic_lines=$diagnostics.Count
        last_diagnostic_lines=@($diagnostics | Select-Object -Last 6 | ForEach-Object Value)
    } | ConvertTo-Json -Depth 5
    [IO.File]::WriteAllText((Join-Path $folder 'diagnostic-proof.json'),$proof,[Text.UTF8Encoding]::new($false))
    Write-Output $json
    Write-Output $proof
} finally {
    if($port.IsOpen){ $port.Close() }
    $port.Dispose()
}
