function LowFields([string]$line){
 $d=@{};foreach($m in [regex]::Matches($line,'(?:^|\s)([a-z0-9_]+)=([^\s]+)')){if($d.ContainsKey($m.Groups[1].Value)){throw 'Duplicate LOW field'};$d[$m.Groups[1].Value]=$m.Groups[2].Value};return $d
}
function ParseLowConfig([string]$line){
 if(-not $line.StartsWith('LOW-CFG ')){throw 'Not LOW config'};$d=LowFields $line
 $expected=@{v='20261019L2';min='10';max='40';step='5';duration='6000';stats='3000';lease='500';kick='3000';kick_min='30';kick_max='200';cap='1500';guard='450';kp10='40';ki10='200';transfer='1500';ff0='470,660';ff10='65,70'}
 foreach($k in $expected.Keys){if($d[$k] -cne $expected[$k]){throw "Unexpected LOW config $k"}}
 if($d.selected -notmatch '^\d+$' -or [int]$d.selected -lt 10 -or [int]$d.selected -gt 40 -or [int]$d.selected%5){throw 'Invalid LOW target'}
 return [pscustomobject]$d
}
function ParseLowState([string]$line){
 if(-not $line.StartsWith('LOW-STATE ')){throw 'Not LOW state'};$d=LowFields $line;$o=[ordered]@{}
 foreach($k in @('ms','mode','fault','id','done','elapsed','motor','target','confirmed','regulating')){if($d[$k] -notmatch '^\d+$'){throw "Missing LOW $k"};$o[$k]=[long]$d[$k]}
 foreach($k in @('enc','pwm','n','sum','e2','ae','min','max')){$m=[regex]::Match($d[$k],'^(-?\d+),(-?\d+)$');if(-not $m.Success){throw "Invalid LOW pair $k"};$o[$k]=@([long]$m.Groups[1].Value,[long]$m.Groups[2].Value)}
 return [pscustomobject]$o
}
function LowOutcome($s,[long]$id,[int]$motor,[int]$target){
 if($s.id -ne $id -or $s.motor -ne $motor -or $s.target -ne $target -or $s.fault -ne 0 -or $s.mode -notin @(0,9)){throw 'LOW identity/fault/mode mismatch'}
 if(($s.confirmed -band (-bnot $motor)) -or ($s.regulating -band (-bnot $s.confirmed))){throw 'Unexpected confirmed/regulating wheel'}
 for($w=0;$w -lt 2;$w++){
  if($motor -band (1 -shl $w)){
   if($s.enc[$w] -lt -5 -or $s.enc[$w] -gt 450 -or $s.pwm[$w] -lt 0 -or $s.pwm[$w] -gt 3000){throw 'Abnormal LOW wheel'}
   if(($s.confirmed -band (1 -shl $w)) -and $s.pwm[$w] -gt 1500){throw 'LOW closed-loop cap exceeded'}
  }elseif([math]::Abs($s.enc[$w]) -gt 10 -or $s.pwm[$w] -ne 0){throw 'Non-selected LOW wheel moved'}
 }
 if($s.mode -eq 9){if($s.done -ne 0){throw 'Active LOW completed'};return 'pending'}
 if($s.done -ne 1 -or $s.elapsed -ne 6000 -or $s.confirmed -ne $motor -or $s.regulating -ne $motor -or $s.pwm[0] -ne 0 -or $s.pwm[1] -ne 0){throw 'LOW incomplete/manual stop'}
 $pass=$true
 for($w=0;$w -lt 2;$w++){
  if($motor -band (1 -shl $w)){
   if($s.n[$w] -ne 300 -or $s.min[$w] -gt $s.max[$w] -or $s.min[$w] -lt -5 -or $s.max[$w] -gt 450 -or $s.e2[$w] -lt 0 -or $s.ae[$w] -lt 0){throw 'Invalid LOW statistics'}
   $mean=$s.sum[$w]/300.0;$rmse=[math]::Sqrt($s.e2[$w]/300.0)
   if($mean -lt $s.min[$w] -or $mean -gt $s.max[$w] -or $rmse+0.00001 -lt [math]::Abs($mean-$target) -or $s.ae[$w]/300.0 -gt $rmse+0.00001){throw 'Inconsistent LOW statistics'}
   if([math]::Abs($mean-$target) -gt [math]::Max(2,$target*.1) -or $rmse -gt [math]::Max(4,$target*.25) -or $s.min[$w] -lt 2 -or $s.max[$w] -gt $target*2){$pass=$false}
  }else{foreach($k in @('n','sum','e2','ae','min','max')){if($s.$k[$w] -ne 0){throw 'Non-selected wheel statistics'}}}
 }
 if($pass){return 'performance_pass'};return 'needs_tuning'
}
