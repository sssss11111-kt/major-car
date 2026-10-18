@echo off
call "D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
set PYTHONDONTWRITEBYTECODE=1
python "%~dp0run_tests.py"
exit /b %errorlevel%
