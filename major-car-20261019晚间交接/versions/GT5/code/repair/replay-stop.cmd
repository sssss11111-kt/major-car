@echo off
call "D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%~dp0"
cl /nologo /std:c11 /O1 /I "../competition/firmware/camera_car/demo/user/inc" replay-stop.c "../competition/firmware/camera_car/demo/user/src/car_core.c" /Fe:replay-stop.exe
if errorlevel 1 exit /b 1
replay-stop.exe "../../records/08-stop-position-images/normal/20261019-213842-648997-01.pgm" "../../records/08-stop-position-images/normal/20261019-213845-251165-02.pgm" "../../records/08-stop-position-images/normal/20261019-213847-855812-03.pgm" > "../../records/08-stop-position-images/replay.txt"
