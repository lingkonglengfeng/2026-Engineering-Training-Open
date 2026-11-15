@echo off
chcp 65001 >nul
title PC Vision - Camera Preview
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0vision\pc_vision\launch.ps1" -Mode preview -NoPause
if errorlevel 1 pause
