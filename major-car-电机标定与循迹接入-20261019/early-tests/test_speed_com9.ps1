param([ValidateSet(1,2,3)][int]$Motor=2)
$side=if($Motor -eq 1){'LEFT'}elseif($Motor -eq 2){'RIGHT'}else{'BOTH'}
$ErrorActionPreference='Stop'
$device=Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq 'COM9'
if(-not $device -or $device.PNPDeviceID -notmatch 'VID_2E3C&PID_5740'){throw 'Wireless USB identity mismatch'}
$now=[TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([DateTime]::UtcNow,'China Standard Time')
$folder=Join-Path $PSScriptRoot ('evidence\'+$side.ToLower()+'-speed-'+$now.ToString('yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$port=[IO.Ports.SerialPort]::new('COM9',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$port.DtrEnable=$false;$port.RtsEnable=$false;$port.WriteTimeout=300
$capture=[IO.MemoryStream]::new();$commands=[Collections.Generic.List[object]]::new()
$failure=$null;$ack=$false;$autoStop=$false;$active=@();$lastBeforeStop=$null
function Send-Command([string]$c){$port.Write($c);$commands.Add(@{utc=[DateTime]::UtcNow.ToString('o');command=$c})}
function Receive-For([int]$ms){
    $timer=[Diagnostics.Stopwatch]::StartNew();$buffer=New-Object byte[] 8192
    while($timer.ElapsedMilliseconds -lt $ms){
        if($port.BytesToRead){$n=$port.Read($buffer,0,[Math]::Min($buffer.Length,$port.BytesToRead));$capture.Write($buffer,0,$n)}
        else{Start-Sleep -Milliseconds 10}
    }
}
function Text-From([int]$offset=0){$b=$capture.ToArray();[Text.Encoding]::ASCII.GetString($b,$offset,$b.Length-$offset)}
function Rows([string]$t){
    $pattern='ms=(\d+) mode=(\d+) fault=(\d+) frames=(\d+)[^\r\n]*bench_target=(\d+) bench_started=(\d+) enc=(-?\d+),(-?\d+) pwm=(\d+),(\d+) us=(\d+) speed=(\d+)[^\r\n]*\r\n'
    foreach($m in [regex]::Matches($t,$pattern)){
        [pscustomobject]@{ms=[long]$m.Groups[1].Value;mode=[int]$m.Groups[2].Value;fault=[int]$m.Groups[3].Value;
          target=[int]$m.Groups[5].Value;started=[int]$m.Groups[6].Value;
          enc_left=[int]$m.Groups[7].Value;enc_right=[int]$m.Groups[8].Value;
          pwm_left=[int]$m.Groups[9].Value;pwm_right=[int]$m.Groups[10].Value}
    }
}
try{
    $port.Open();$port.DiscardInBuffer();Send-Command 'p';Send-Command 'q';Receive-For 1800
    $text=Text-From;$rows=@(Rows $text)
    if(-not $rows.Count){throw 'No complete preflight status'}
    $s=$rows[-1]
    if($s.mode -ne 0 -or $s.fault -ne 0 -or $s.pwm_left -ne 0 -or $s.pwm_right -ne 0){throw 'Require idle, no fault, zero outputs'}
    if($text -notmatch 'UART8-DIAG[^\r\n]*diag=0 ab=1 ' -or $text -notmatch 'MOTOR-DIR v=20261017I left=C11 cfg=1 sample=[01] right=C9 cfg=0'){
        throw 'Right speed bench camera firmware not confirmed; no motion command sent'
    }
    $offset=[int]$capture.Length;Send-Command $(if($Motor -eq 1){'4'}elseif($Motor -eq 2){'5'}else{'6'})
    $timer=[Diagnostics.Stopwatch]::StartNew();$nextQuery=0;$lastReceived=0;$lastMs=-1
    while($timer.ElapsedMilliseconds -lt 4700){
        if($timer.ElapsedMilliseconds -ge $nextQuery){Send-Command 'p';$nextQuery+=200}
        Receive-For 40;$rows=@(Rows (Text-From $offset))
        if($rows.Count){
            $s=$rows[-1]
            if($s.ms -ne $lastMs){$lastMs=$s.ms;$lastReceived=$timer.ElapsedMilliseconds}
            if($Motor -eq 3 -and ($s.enc_left -lt -5 -or $s.enc_left -gt 450 -or $s.enc_right -lt -5 -or $s.enc_right -gt 450)){
                Send-Command 's';throw 'Abnormal dual-wheel feedback; stop sent'
            }
            $feedback=if($Motor -eq 1){$s.enc_left}else{$s.enc_right}
            $other=if($Motor -eq 3){0}elseif($Motor -eq 1){$s.enc_right}else{$s.enc_left}
            $otherPwm=if($Motor -eq 3){0}elseif($Motor -eq 1){$s.pwm_right}else{$s.pwm_left}
            if($s.fault -ne 0 -or $s.mode -notin @(0,4) -or $otherPwm -ne 0 -or $s.pwm_left -gt 3000 -or $s.pwm_right -gt 3000 -or $feedback -lt -5 -or $feedback -gt 450 -or [Math]::Abs($other) -gt 10){
                Send-Command 's';throw 'Abnormal mode, output or encoder feedback; stop sent'
            }
        }
        if($timer.ElapsedMilliseconds-$lastReceived -gt 700){Send-Command 's';throw 'Telemetry stale; stop sent'}
    }
    Send-Command 'p';Receive-For 250;$motion=Text-From $offset;$rows=@(Rows $motion)
    $ack=$motion.Contains("$side SPEED TEST v=20261017I 4s target=80 cap=30%")
    $active=@($rows | Where-Object mode -eq 4);$lastBeforeStop=$rows[-1]
    $autoStop=$lastBeforeStop.mode -eq 0 -and $lastBeforeStop.fault -eq 0 -and $lastBeforeStop.pwm_left -eq 0 -and $lastBeforeStop.pwm_right -eq 0
    if(-not $ack -or -not $active.Count -or -not $autoStop){throw 'Test ACK, active bench samples or automatic zero-output proof missing'}
}catch{$failure=$_.Exception.Message}
finally{
    if($port.IsOpen){try{Send-Command 's';Send-Command 'p';Receive-For 400}catch{};$port.Close()};$port.Dispose()
    $raw=$capture.ToArray();[IO.File]::WriteAllBytes((Join-Path $folder 'serial.bin'),$raw)
    $all=@(Rows ([Text.Encoding]::ASCII.GetString($raw)));$capture.Dispose()
    $all | Export-Csv -LiteralPath (Join-Path $folder 'status.csv') -NoTypeInformation -Encoding UTF8
    $result=[ordered]@{time_china=$now.ToString('yyyy-MM-dd HH:mm:ss');motor=$Motor;failure=$failure;ack=$ack;automatic_stop=$autoStop;
      active_samples=$active;last_before_explicit_stop=$lastBeforeStop;final=$(if($all.Count){$all[-1]}else{$null});
      physical_observation='pending';commands=@($commands.ToArray());evidence=$folder}
    $json=$result | ConvertTo-Json -Depth 6
    [IO.File]::WriteAllText((Join-Path $folder 'result.json'),$json,[Text.UTF8Encoding]::new($false));$json
}
if($failure){exit 1}
