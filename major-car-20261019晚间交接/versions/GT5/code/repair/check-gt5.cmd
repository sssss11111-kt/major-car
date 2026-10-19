@echo off
call "D:\Program file\visual studio\VC\Auxiliary\Build\vcvars64.bat" >nul
python "%~dp0run_tests.py"
