@echo off
chcp 65001 >nul
title PC Vision - Desktop Panel
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0vision\pc_vision\launch.ps1" -Mode run -NoPause
if errorlevel 1 pause
