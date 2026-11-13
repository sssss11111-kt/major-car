param([string]$Destination = "$env:TEMP/crossing-2.dll")
$root = $PSScriptRoot
$p = "$root/deliveries/lap30-ring-20261107/project"
$u = "$p/demo/user"
$s = "$root/firmware/camera_car/demo/user"
$v = "$p/verification"
$zig = if ($env:ZIG) { $env:ZIG } else { 'D:/major-car/firmware/camera_car/.tools/ziglang/zig.exe' }
& $zig cc -shared -O2 -DR17_WITH_SPECIAL_TRACK -DRB_TURN_SIGN=1 -I "$v/vision-stubs" -I "$u/inc" -I "$s/inc" "$u/src/img_process.c" "$u/src/steering_control.c" "$u/src/cross_drive.c" "$u/src/roundabout_scan.c" "$s/src/special_track.c" "$s/src/special_track_scan.c" "$s/src/special_route.c" "$s/src/cross_island.c" "$s/src/roundabout.c" "$v/vision-replay-wrapper.c" -o $Destination
exit $LASTEXITCODE
