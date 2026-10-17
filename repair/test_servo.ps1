param([ValidateSet('center','low','high')][string]$Action='center')
$ErrorActionPreference='Stop'
$command=if($Action -eq 'center'){'c'}elseif($Action -eq 'low'){'l'}else{'j'}
$expectedUs=if($Action -eq 'center'){1500}elseif($Action -eq 'low'){1350}else{1650}
$device=Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq 'COM6'
if(-not $device -or $device.PNPDeviceID -notmatch 'VID_2E3C&PID_5740'){throw 'Wireless USB identity mismatch'}
$now=[TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([DateTime]::UtcNow,'China Standard Time')
$folder=Join-Path $PSScriptRoot ('evidence\servo-'+$now.ToString('yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$port=[IO.Ports.SerialPort]::new('COM6',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$port.DtrEnable=$false;$port.RtsEnable=$false;$port.WriteTimeout=300
$capture=[IO.MemoryStream]::new();$sent=[Collections.Generic.List[object]]::new();$failure=$null
$active=@();$returned=$false;$lastBeforeStop=$null
function Send-Command([string]$c){$port.Write($c);$sent.Add(@{utc=[DateTime]::UtcNow.ToString('o');command=$c})}
function Receive-For([int]$ms){
    $timer=[Diagnostics.Stopwatch]::StartNew();$buffer=New-Object byte[] 8192
    while($timer.ElapsedMilliseconds -lt $ms){
        if($port.BytesToRead){$n=$port.Read($buffer,0,[Math]::Min($buffer.Length,$port.BytesToRead));$capture.Write($buffer,0,$n)}
        else{Start-Sleep -Milliseconds 10}
    }
}
function Text-From([int]$offset=0){$b=$capture.ToArray();[Text.Encoding]::ASCII.GetString($b,$offset,$b.Length-$offset)}
function Rows([string]$t){
    foreach($m in [regex]::Matches($t,'ms=(\d+) mode=(\d+) fault=(\d+)[^\r\n]*enc=(-?\d+),(-?\d+) pwm=(\d+),(\d+) us=(\d+) speed=(\d+)\r\n')){
        [pscustomobject]@{ms=[long]$m.Groups[1].Value;mode=[int]$m.Groups[2].Value;fault=[int]$m.Groups[3].Value;
          enc_left=[int]$m.Groups[4].Value;enc_right=[int]$m.Groups[5].Value;
          pwm_left=[int]$m.Groups[6].Value;pwm_right=[int]$m.Groups[7].Value;us=[int]$m.Groups[8].Value}
    }
}
try{
    $port.Open();$port.DiscardInBuffer();Send-Command 'p';Send-Command 'q';Receive-For 1500
    $text=Text-From;$rows=@(Rows $text)
    if(-not $rows.Count){throw 'No complete preflight status'}
    $s=$rows[-1]
    if($s.mode -ne 0 -or $s.fault -ne 0 -or $s.pwm_left -ne 0 -or $s.pwm_right -ne 0){throw 'Require idle, no fault, zero motor outputs'}
    if($text -notmatch 'UART8-DIAG[^\r\n]*diag=0 ab=1 ' -or $text -notmatch 'MOTOR-DIR v=20261017I '){throw 'Expected camera firmware not confirmed'}
    $offset=[int]$capture.Length;Send-Command $command
    $timer=[Diagnostics.Stopwatch]::StartNew();$nextQuery=0
    while($timer.ElapsedMilliseconds -lt 1500){
        if($timer.ElapsedMilliseconds -ge $nextQuery){Send-Command 'p';$nextQuery+=100}
        Receive-For 35;$rows=@(Rows (Text-From $offset))
        if($rows.Count){
            $s=$rows[-1]
            if($s.mode -ne 0 -or $s.fault -ne 0 -or $s.pwm_left -ne 0 -or $s.pwm_right -ne 0 -or $s.us -lt 1350 -or $s.us -gt 1650){
                Send-Command 's';throw 'Unexpected motor output, mode, fault or servo command; stop sent'
            }
        }
    }
    Send-Command 'p';Receive-For 200;$rows=@(Rows (Text-From $offset))
    $active=@($rows | Where-Object us -eq $expectedUs);$lastBeforeStop=$rows[-1]
    $returned=$lastBeforeStop.us -eq 1500 -and $lastBeforeStop.pwm_left -eq 0 -and $lastBeforeStop.pwm_right -eq 0
    if(-not $active.Count -or -not $returned){throw 'Requested servo command or automatic center evidence missing'}
}catch{$failure=$_.Exception.Message}
finally{
    if($port.IsOpen){try{Send-Command 's';Send-Command 'p';Receive-For 350}catch{};$port.Close()};$port.Dispose()
    $raw=$capture.ToArray();[IO.File]::WriteAllBytes((Join-Path $folder 'serial.bin'),$raw)
    $all=@(Rows ([Text.Encoding]::ASCII.GetString($raw)));$capture.Dispose()
    $all | Export-Csv -LiteralPath (Join-Path $folder 'status.csv') -NoTypeInformation -Encoding UTF8
    $r=[ordered]@{action=$Action;expected_us=$expectedUs;failure=$failure;command_samples=$active;automatic_center_observed=$returned;
       last_before_explicit_stop=$lastBeforeStop;final=$(if($all.Count){$all[-1]}else{$null});physical_observation='pending';commands=@($sent.ToArray());evidence=$folder}
    $json=$r | ConvertTo-Json -Depth 5
    [IO.File]::WriteAllText((Join-Path $folder 'result.json'),$json,[Text.UTF8Encoding]::new($false));$json
}
if($failure){exit 1}
