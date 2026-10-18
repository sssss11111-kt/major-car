param([ValidateSet(1,2)][int]$Motor=1,[ValidateRange(1,3)][int]$Pulses=1,[string]$PortName='COM6')
$ErrorActionPreference='Stop'
$expectedPwm=3000
$durationMs=2000
$expectedAck=if($Motor -eq 1){'LEFT MOTOR TEST 2s at 30%'}else{'RIGHT MOTOR TEST 2s at 30%'}
$device=Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq $PortName
if(-not $device -or $device.PNPDeviceID -notmatch 'VID_2E3C&PID_5740'){throw 'Wrong wireless USB port'}
$now=[TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([DateTime]::UtcNow,'China Standard Time')
$folder=Join-Path $PSScriptRoot ('evidence\motor-'+$now.ToString('yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$port=New-Object System.IO.Ports.SerialPort $PortName,115200,None,8,One
$port.DtrEnable=$false;$port.RtsEnable=$false;$port.ReadTimeout=100;$port.WriteTimeout=300
$capture=New-Object IO.MemoryStream
$sent=New-Object 'System.Collections.Generic.List[object]'
$results=New-Object 'System.Collections.Generic.List[object]'
$failure=$null
function Send-Command([string]$Command){
    $raw=[Text.Encoding]::ASCII.GetBytes($Command)
    $port.Write($raw,0,$raw.Length)
    $sent.Add([ordered]@{utc=[DateTime]::UtcNow.ToString('o');command=$Command})
}
function Receive-For([int]$Milliseconds){
    $watch=[Diagnostics.Stopwatch]::StartNew();$buffer=New-Object byte[] 8192
    while($watch.ElapsedMilliseconds -lt $Milliseconds){
        if($port.BytesToRead -gt 0){
            $n=$port.Read($buffer,0,[Math]::Min($buffer.Length,$port.BytesToRead))
            $capture.Write($buffer,0,$n)
        } else {Start-Sleep -Milliseconds 10}
    }
}
function All-Text {return [Text.Encoding]::ASCII.GetString($capture.ToArray())}
function Status-Rows([string]$Text){
    $pattern='ms=(\d+) mode=(\d+) fault=(\d+) frames=(\d+)[^\r\n]*?enc=(-?\d+),(-?\d+) pwm=(\d+),(\d+) us=(\d+) speed=(\d+)\r\n'
    foreach($m in [regex]::Matches($Text,$pattern)){
        [pscustomobject]@{ms=[long]$m.Groups[1].Value;mode=[int]$m.Groups[2].Value;
            fault=[int]$m.Groups[3].Value;frames=[long]$m.Groups[4].Value;
            enc_left=[int]$m.Groups[5].Value;enc_right=[int]$m.Groups[6].Value;
            pwm_left=[int]$m.Groups[7].Value;pwm_right=[int]$m.Groups[8].Value}
    }
}
try {
    $port.Open();$port.DiscardInBuffer()
    Send-Command 'p';Send-Command 'q';Receive-For 1200
    $text=All-Text;$rows=@(Status-Rows $text)
    if(-not $rows.Count){throw 'No complete preflight status'}
    $last=$rows[-1]
    if($last.mode -ne 0 -or $last.fault -ne 0 -or $last.pwm_left -ne 0 -or $last.pwm_right -ne 0){throw 'Preflight requires idle, no fault, zero PWM'}
    if($text -notmatch 'UART8-DIAG v=20261017B[^\r\n]*diag=0 ab=1 '){throw 'Expected verified camera firmware/baud ACK not confirmed'}
    if($text -notmatch 'MOTOR-DIR v=20261017I left=C11 cfg=1 sample=[01] right=C9 cfg=0 sample=[01] sample_unverified=1'){
        throw 'Expected C11 left direction camera firmware not confirmed; no motion command sent'
    }
    for($pulse=1;$pulse -le $Pulses;$pulse++){
        $start=$capture.Length
        Send-Command ([string]$Motor)
        $watch=[Diagnostics.Stopwatch]::StartNew();$nextQuery=0
        while($watch.ElapsedMilliseconds -lt ($durationMs+500)){
            if($watch.ElapsedMilliseconds -ge $nextQuery){Send-Command 'p';$nextQuery+=150}
            Receive-For 40
            $data=$capture.ToArray();$motion=[Text.Encoding]::ASCII.GetString($data,[int]$start,$data.Length-[int]$start)
            $samples=@(Status-Rows $motion)
            if($samples.Count){
                $s=$samples[-1]
                if($s.fault -ne 0 -or $s.mode -eq 1 -or $s.pwm_left -gt $expectedPwm -or $s.pwm_right -gt $expectedPwm -or
                   ($Motor -eq 1 -and $s.pwm_right -ne 0) -or ($Motor -eq 2 -and $s.pwm_left -ne 0)){
                    Send-Command 's';throw 'Unexpected fault, running mode, PWM or opposite-wheel output; stopped'
                }
            }
        }
        Send-Command 'p';Receive-For 250
        $data=$capture.ToArray();$motion=[Text.Encoding]::ASCII.GetString($data,[int]$start,$data.Length-[int]$start)
        $samples=@(Status-Rows $motion)
        $active=@($samples | Where-Object { $_.mode -eq 2 -and
            (($Motor -eq 1 -and $_.pwm_left -eq $expectedPwm -and $_.pwm_right -eq 0) -or
             ($Motor -eq 2 -and $_.pwm_right -eq $expectedPwm -and $_.pwm_left -eq 0)) })
        $last=$samples[-1]
        $stopped=$last.mode -eq 0 -and $last.fault -eq 0 -and $last.pwm_left -eq 0 -and $last.pwm_right -eq 0
        $accepted=$motion.Contains($expectedAck)
        $results.Add([ordered]@{pulse=$pulse;motor=$Motor;requested_pwm_percent=($expectedPwm/100);firmware_duration_ms=$durationMs;
            command_ack=$accepted;active_samples=$active;last_status_before_explicit_stop=$last;
            automatic_stop_observed=$stopped;physical_rotation='Requires user observation'})
        Send-Command 's';Send-Command 'p';Receive-For 350
        if(-not $accepted -or -not $active.Count -or -not $stopped){throw 'Motor test ACK, active PWM or automatic-stop evidence missing'}
        Receive-For 300
    }
} catch {$failure=$_.Exception.Message}
finally {
    if($port.IsOpen){
        try{Send-Command 's';Send-Command 'p';Receive-For 300}catch{}
        $port.Close()
    }
    $port.Dispose()
    [IO.File]::WriteAllBytes((Join-Path $folder 'serial.bin'),$capture.ToArray())
    $allRows=@(Status-Rows (All-Text));$capture.Dispose()
    if($allRows.Count){$allRows | Export-Csv -LiteralPath (Join-Path $folder 'status.csv') -NoTypeInformation -Encoding UTF8}
    $summary=[ordered]@{time_china=$now.ToString('yyyy-MM-dd HH:mm:ss');port=$PortName;
        motor=$Motor;pulses=$Pulses;results=@($results.ToArray());failure=$failure;
        final_status=$(if($allRows.Count){$allRows[-1]}else{$null});commands=@($sent.ToArray());evidence=$folder}
    $json=$summary|ConvertTo-Json -Depth 10
    [IO.File]::WriteAllText((Join-Path $folder 'result.json'),$json,[Text.UTF8Encoding]::new($false))
    Write-Output $json
}
if($failure){exit 1}
