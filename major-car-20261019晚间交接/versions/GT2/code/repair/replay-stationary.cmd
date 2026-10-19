@echo off
call "D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%~dp0"
cl /nologo /std:c11 /O1 /I "../competition/firmware/camera_car/demo/user/inc" replay-static.c "../competition/firmware/camera_car/demo/user/src/car_core.c" /Fe:replay-static.exe
if errorlevel 1 exit /b 1
replay-static.exe "../../records/09-stationary/gray0.pgm"
replay-static.exe "../../records/09-stationary/gray1.pgm"
replay-static.exe "../../records/09-stationary/gray2.pgm"
