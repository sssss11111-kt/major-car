@echo off
call "D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0..\code"
cl /nologo /std:c11 /W3 /O1 /D_CRT_SECURE_NO_WARNINGS /I competition\firmware\camera_car\demo\user\inc competition\firmware\camera_car\tests\test_tracking.c competition\firmware\camera_car\demo\user\src\car_core.c /Fe:repair\evidence\test_tracking.exe /Fo:repair\evidence\
if errorlevel 1 exit /b 1
repair\evidence\test_tracking.exe
