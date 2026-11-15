@echo off
chcp 65001 >nul
title PC Vision - Environment Check
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0vision\pc_vision\launch.ps1" -Mode check -NoPause
pause
