@echo off
if not defined VS_VCVARS64 set "VS_VCVARS64=D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat"
call "%VS_VCVARS64%" >nul
if errorlevel 1 exit /b 1
python "%~dp0run-host-checks-msvc.py"
exit /b %errorlevel%
