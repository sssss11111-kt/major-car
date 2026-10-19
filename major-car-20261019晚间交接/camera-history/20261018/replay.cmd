@echo off
call "D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%~dp0"
cl /nologo /std:c11 /O1 /D_CRT_SECURE_NO_WARNINGS /I "D:\桌面\智能车\.tmp\review-n-53d6dba\competition\firmware\camera_car\demo\user\inc" replay.c "D:\桌面\智能车\.tmp\review-n-53d6dba\competition\firmware\camera_car\demo\user\src\car_core.c" /Fe:replay-n.exe
if errorlevel 1 exit /b 1
replay-n.exe gray.pgm result-n.csv
replay-n.exe gray-repeat.pgm result-n-repeat.csv
cl /nologo /std:c11 /O1 /D_CRT_SECURE_NO_WARNINGS /I "D:\桌面\智能车\major-car-低速循迹整合-20261018\code\competition\firmware\camera_car\demo\user\inc" replay.c "D:\桌面\智能车\major-car-低速循迹整合-20261018\code\competition\firmware\camera_car\demo\user\src\car_core.c" /Fe:replay-t.exe
if errorlevel 1 exit /b 1
replay-t.exe gray.pgm result-t.csv
replay-t.exe gray-repeat.pgm result-t-repeat.csv
