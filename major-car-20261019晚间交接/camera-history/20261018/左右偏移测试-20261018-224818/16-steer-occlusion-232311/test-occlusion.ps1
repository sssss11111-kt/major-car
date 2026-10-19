$ErrorActionPreference='Stop'
$folder=$PSScriptRoot
$port=[IO.Ports.SerialPort]::new('COM6',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$port.ReadTimeout=100;$port.WriteTimeout=500;$port.DtrEnable=$false;$port.RtsEnable=$false
$raw=[IO.MemoryStream]::new();$text='';$rows=[Collections.Generic.List[object]]::new();$armed=$false
function Receive-Checked([int]$duration){
 $timer=[Diagnostics.Stopwatch]::StartNew();$buffer=[byte[]]::new(4096)
 while($timer.ElapsedMilliseconds -lt $duration){
  $available=$port.BytesToRead
  if($available){
   $n=$port.Read($buffer,0,[Math]::Min($available,$buffer.Length));$raw.Write($buffer,0,$n);$script:text += [Text.Encoding]::ASCII.GetString($buffer,0,$n)
   while($script:text.Contains("`n")){
    $end=$script:text.IndexOf("`n");$line=$script:text.Substring(0,$end).Trim();$script:text=$script:text.Substring($end+1)
    Add-Content -LiteralPath (Join-Path $folder 'serial.txt') -Value $line -Encoding utf8
    if($line -match '^ms=(\d+) mode=(\d+) fault=(\d+) frames=(\d+) valid=(\d+) pair=(\d+)/(\d+).*err10=(-?\d+).*enc=(-?\d+),(-?\d+) pwm=(\d+),(\d+) us=(\d+)'){
     $row=[pscustomobject]@{ms=[int64]$Matches[1];mode=[int]$Matches[2];fault=[int]$Matches[3];frames=[int]$Matches[4];valid=[int]$Matches[5];paired=[int]$Matches[6];scanned=[int]$Matches[7];err10=[int]$Matches[8];left=[int]$Matches[9];right=[int]$Matches[10];pwm_left=[int]$Matches[11];pwm_right=[int]$Matches[12];us=[int]$Matches[13]}
     $rows.Add($row)
     if($row.pwm_left -ne 0 -or $row.pwm_right -ne 0 -or $row.mode -notin @(0,3,5) -or ($row.fault -ne 0 -and -not ($row.mode -eq 3 -and $row.fault -in @(2,3)))){$port.Write('s');throw 'Unexpected motion mode/output/fault; stop sent'}
    }
   }
  }else{Start-Sleep -Milliseconds 10}
 }
}
try{
 $port.Open();$port.Write('s');Start-Sleep -Milliseconds 100;$port.DiscardInBuffer();$port.Write('dp');Receive-Checked 1800
 if($rows.Count -lt 3){throw 'Insufficient fresh status; w not sent'}
 $pre=@($rows | Select-Object -Last 3)
 if(@($pre | Where-Object {$_.mode -ne 0 -or $_.valid -ne 1 -or $_.us -ne 1500}).Count -or $pre[-1].frames -le $pre[0].frames){throw 'Preflight requires idle, neutral and growing valid frames; w not sent'}
 if(($pre[-1].ms-$pre[0].ms) -lt 200){throw 'Preflight interval too short; w not sent'}
 $start=$rows.Count;$armed=$true;$port.Write('w')
 for($i=0;$i -lt 48;$i++){$port.Write('p');Receive-Checked 500; if($i -eq 3){Write-Output 'STATIC_STEER_ACTIVE_NOW_COVER'} }
 $active=@($rows | Select-Object -Skip $start | Where-Object mode -eq 5)
 $port.Write('s');$port.Write('p');Receive-Checked 1000
 $last=$rows[-1]
 $result=[ordered]@{commands=@('s','d','p','w','p queries','s','p');preflight=$pre;active_samples=$active.Count;active_first=$(if($active.Count){$active[0]}else{$null});active_last=$(if($active.Count){$active[-1]}else{$null});active_us=@($active|ForEach-Object us|Sort-Object -Unique);active_err10=@($active|ForEach-Object err10|Sort-Object -Unique);final=$last;stopped=($last.mode -eq 0 -and $last.pwm_left -eq 0 -and $last.pwm_right -eq 0 -and $last.us -eq 1500);physical_observation='pending user confirmation'}
 $result|ConvertTo-Json -Depth 6|Set-Content -LiteralPath (Join-Path $folder 'result.json') -Encoding utf8
 $result|ConvertTo-Json -Depth 6|Write-Output
 if(-not $active.Count){throw 'No static steering mode observed'}
 if(-not $result.stopped){throw 'Final stop not verified'}
}finally{
 if($port.IsOpen){try{$port.Write('s')}catch{};$port.Close()};$port.Dispose();[IO.File]::WriteAllBytes((Join-Path $folder 'serial.bin'),$raw.ToArray());$raw.Dispose()
 $rows|ConvertTo-Json -Depth 4|Set-Content -LiteralPath (Join-Path $folder 'status.json') -Encoding utf8
}

