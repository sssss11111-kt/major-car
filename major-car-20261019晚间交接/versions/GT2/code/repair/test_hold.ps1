param([ValidateSet(1,2)][int]$Motor=1,[ValidateRange(0,30)][int]$DutyPercent=0,[string]$PortName='COM9',[switch]$Run)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'hold_protocol.ps1')
$device=Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq $PortName
if(-not $device -or $device.PNPDeviceID -notmatch 'VID_2E3C&PID_5740'){throw 'Wrong wireless USB identity'}
$stamp=[TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([DateTime]::UtcNow,'China Standard Time').ToString('yyyyMMdd-HHmmss')
$folder=Join-Path $PSScriptRoot ('evidence\hold-'+$stamp)
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$port=[IO.Ports.SerialPort]::new($PortName,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$port.DtrEnable=$false;$port.RtsEnable=$false;$port.WriteTimeout=300
$raw=[IO.MemoryStream]::new();$commands=[Collections.Generic.List[object]]::new()
$failure=$null;$outcome='preflight_only';$completion=$null;$cfg=$null;$pre=$null;$trialId=$null
$seenKick=$false;$seenHold=$false;$duty=$DutyPercent*100
function Send([string]$c){$port.Write($c);$commands.Add(@{utc=[DateTime]::UtcNow.ToString('o');command=$c})}
function Receive([int]$ms){
    $watch=[Diagnostics.Stopwatch]::StartNew();$buffer=New-Object byte[] 8192
    while($watch.ElapsedMilliseconds -lt $ms){
        if($port.BytesToRead){$n=$port.Read($buffer,0,[Math]::Min($buffer.Length,$port.BytesToRead));$raw.Write($buffer,0,$n)}
        else{Start-Sleep -Milliseconds 5}
    }
}
function Text([int]$offset=0){$b=$raw.ToArray();return [Text.Encoding]::ASCII.GetString($b,$offset,$b.Length-$offset)}
function States([string]$text){foreach($m in [regex]::Matches($text,'HOLD-STATE [^\r\n]*\r\n')){Parse-HoldState $m.Value}}
function Config([string]$text){$m=[regex]::Matches($text,'HOLD-CFG [^\r\n]*\r\n');if(-not $m.Count){throw 'No complete H1 config'};return Parse-HoldConfig $m[-1].Value}
function Idle($s){
    if(-not $s -or $s.mode -ne 0 -or $s.fault -ne 0 -or $s.pwm_left -ne 0 -or $s.pwm_right -ne 0 -or $s.enc_left -ne 0 -or $s.enc_right -ne 0){throw 'Require idle and zero output/feedback'}
}
try{
    $port.Open();$port.DiscardInBuffer();Send 'H';Receive 650;Send 'q';Receive 500;Send 'H';Receive 650
    $text=Text;$cfg=Config $text;$states=@(States $text);$pre=$states[-1];Idle $pre
    if($text -notmatch 'UART8-DIAG[^\r\n]*diag=0 ab=1 ' -or $text -notmatch 'MOTOR-DIR v=20261017I left=C11 cfg=1 sample=[01] right=C9 cfg=0'){throw 'Verified wireless/direction firmware not confirmed'}
    if($Run){
        Send ('K'*30);Receive 400;if($DutyPercent){Send ('I'*$DutyPercent);Receive 400}
        $offset=[int]$raw.Length;Send 'H';Receive 650;$setup=Text $offset;$cfg=Config $setup
        if($cfg.selected -ne $duty){throw 'Selected hold duty not acknowledged'}
        $pre=@(States $setup)[-1];Idle $pre;$previousId=$pre.id
        $offset=[int]$raw.Length;Send $(if($Motor -eq 1){'U'}else{'V'})
        $clock=[Diagnostics.Stopwatch]::StartNew();$next=0;$lastRx=0;$lastMs=-1
        while($clock.ElapsedMilliseconds -lt 4600){
            if($clock.ElapsedMilliseconds -ge $next){Send 'H';$next+=100}
            Receive 20;$motion=Text $offset
            if($motion.Contains('HOLD-START REFUSED')){throw 'Hold start refused'}
            $states=@(States $motion)
            if($states.Count){
                $s=$states[-1]
                if($s.ms -ne $lastMs){$lastMs=$s.ms;$lastRx=$clock.ElapsedMilliseconds}
                if($s.id -ne $previousId){
                    if($null -eq $trialId){$trialId=$s.id}
                    $candidate=Get-HoldOutcome $s $trialId $Motor $duty
                    $active=if($Motor -eq 1){$s.enc_left}else{$s.enc_right}
                    $pwm=if($Motor -eq 1){$s.pwm_left}else{$s.pwm_right}
                    if($s.phase -eq 1 -and $active -ge 5 -and $pwm -eq 3000){$seenKick=$true}
                    if($s.phase -eq 2){$seenHold=$true;if($s.kick_ticks -eq 3){$seenKick=$true}}
                    if($candidate -ne 'pending'){$outcome=$candidate;$completion=$s;break}
                }
            }
            if($clock.ElapsedMilliseconds-$lastRx -gt 700){throw 'Hold telemetry stale'}
        }
        $ack="HOLD-START v=20261019H1 motor=$Motor duty=$duty kick=3000 kick_min=200 kick_max=300 hold=3000 lease=500 guard=1500"
        if(-not $completion -or -not $seenKick -or -not $seenHold -or -not (Text $offset).Contains($ack)){throw 'Missing identified kick/drop/automatic outcome evidence'}
    }
}catch{$failure=$_.Exception.Message;$outcome='error'}
finally{
    if($port.IsOpen){try{Send 's';Send 'H';Receive 450}catch{};$port.Close()};$port.Dispose()
    [IO.File]::WriteAllBytes((Join-Path $folder 'serial.bin'),$raw.ToArray());$all=@(States (Text));$raw.Dispose()
    $all | Export-Csv -LiteralPath (Join-Path $folder 'status.csv') -NoTypeInformation -Encoding UTF8
    $r=[ordered]@{version='20261019H1';motor=$Motor;duty_percent=$DutyPercent;failure=$failure;outcome=$outcome;
        measurement_complete=($null -ne $completion -and -not $failure);continuous_feedback=($outcome -eq 'continuous');
        kick_observed=$seenKick;hold_observed=$seenHold;completion=$completion;configuration=$cfg;
        final=$(if($all.Count){$all[-1]}else{$null});commands=@($commands.ToArray());physical_observation='pending';evidence=$folder}
    $r | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $folder 'result.json') -Encoding utf8
    [pscustomobject]@{motor=$Motor;duty_percent=$DutyPercent;outcome=$outcome;failure=$failure;completion=$completion;final=$r.final;evidence=$folder} | ConvertTo-Json -Depth 4
}
if($failure){exit 1}
exit 0
