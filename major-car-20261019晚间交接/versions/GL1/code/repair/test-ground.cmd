@echo off
call "D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%~dp0"
cl /nologo /std:c11 /W3 /O1 /UNDEBUG /I "../competition/firmware/camera_car/demo/user/inc" "../competition/firmware/camera_car/tests/test_ground.c" "../competition/firmware/camera_car/demo/user/src/car_core.c" /Fe:test-ground.exe
if errorlevel 1 exit /b 1
test-ground.exe


