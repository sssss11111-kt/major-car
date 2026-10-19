function Hold-Fields([string]$Line){
    $d=@{};foreach($m in [regex]::Matches($Line,'(?:^|\s)([a-z_]+)=([^\s]+)')){
        $k=$m.Groups[1].Value;if($d.ContainsKey($k)){throw 'Duplicate hold field'};$d[$k]=$m.Groups[2].Value
    };return $d
}
function Parse-HoldConfig([string]$Line){
    if(-not $Line.StartsWith('HOLD-CFG ')){throw 'Not a hold configuration'}
    $d=Hold-Fields $Line
    if($d.v -ne '20261019H1'){throw 'Unknown hold firmware'}
    $expected=@{step=100;cap=3000;kick=3000;kick_min=200;kick_max=300;hold=3000;lease=500;guard=1500}
    foreach($k in $expected.Keys){if($d[$k] -notmatch '^\d+$' -or [long]$d[$k] -ne $expected[$k]){throw "Unexpected hold config: $k"}}
    if($d.selected -notmatch '^\d+$' -or [int]$d.selected -gt 3000 -or [int]$d.selected%100 -ne 0){throw 'Invalid hold selection'}
    $o=[ordered]@{v=$d.v;selected=[int]$d.selected};foreach($k in $expected.Keys){$o[$k]=$expected[$k]}
    return [pscustomobject]$o
}
function Parse-HoldState([string]$Line){
    if(-not $Line.StartsWith('HOLD-STATE ')){throw 'Not a hold state'}
    $d=Hold-Fields $Line;$o=[ordered]@{}
    foreach($k in @('ms','mode','fault','id','phase','result','total_ms','hold_ms','motor','duty','n','sum','min','max','zeros','kick_ticks')){
        if($d[$k] -notmatch '^-?\d+$'){throw "Missing hold state: $k"};$o[$k]=[long]$d[$k]
    }
    foreach($k in @('enc','pwm')){
        $m=[regex]::Match($d[$k],'^(-?\d+),(-?\d+)$');if(-not $m.Success){throw 'Invalid hold pair'}
        $o[$k+'_left']=[long]$m.Groups[1].Value;$o[$k+'_right']=[long]$m.Groups[2].Value
    }
    return [pscustomobject]$o
}
function Get-HoldOutcome($State,[long]$Id,[int]$Motor,[int]$Duty){
    $s=$State
    if($s.id -ne $Id -or $s.motor -ne $Motor -or $s.duty -ne $Duty){throw 'Hold identity/side/duty mismatch'}
    if($s.fault -ne 0 -or $s.mode -notin @(0,8)){throw 'Hold fault or unexpected mode'}
    $active=if($Motor -eq 1){$s.enc_left}else{$s.enc_right}
    $other=if($Motor -eq 1){$s.enc_right}else{$s.enc_left}
    $pwm=if($Motor -eq 1){$s.pwm_left}else{$s.pwm_right}
    $otherPwm=if($Motor -eq 1){$s.pwm_right}else{$s.pwm_left}
    if($active -lt -5 -or $active -gt 1500 -or [math]::Abs($other) -gt 10 -or $pwm -lt 0 -or $pwm -gt 3000 -or $otherPwm -ne 0){throw 'Abnormal hold feedback or output'}
    if($s.kick_ticks -lt 0 -or $s.kick_ticks -gt 3 -or ($s.phase -in @(2,3) -and $s.kick_ticks -ne 3)){throw 'Kick confirmation missing'}
    if($s.phase -in @(1,2)){
        if($s.mode -ne 8 -or $s.result -ne 0){throw 'Invalid active hold state'}
        if($s.phase -eq 1 -and $pwm -ne 3000 -and -not ($s.total_ms -eq 0 -and $pwm -eq 0)){throw 'Unexpected kick output'}
        if($s.phase -eq 2 -and $pwm -ne $Duty){throw 'Unexpected sustain output'}
        return 'pending'
    }
    if($s.phase -ne 3 -or $s.mode -ne 0 -or $s.pwm_left -ne 0 -or $s.pwm_right -ne 0){throw 'Interrupted hold is not a completed measurement'}
    if($s.result -eq 2){
        if($s.hold_ms -lt 500 -or $s.hold_ms -ge 3000){throw 'Invalid lost-motion timing'}
        return 'lost_motion'
    }
    if($s.result -ne 1 -or $s.hold_ms -ne 3000 -or $s.total_ms -lt 3200 -or $s.total_ms -gt 3300 -or $s.n -ne 200){throw 'Incomplete hold window'}
    $mean=$s.sum/[double]$s.n
    if($s.min -gt $s.max -or $mean -lt $s.min -or $mean -gt $s.max -or $s.zeros -lt 0 -or $s.zeros -gt $s.n){throw 'Invalid hold statistics'}
    if($s.min -ge 2 -and $s.zeros -eq 0){return 'continuous'}
    return 'intermittent'
}
