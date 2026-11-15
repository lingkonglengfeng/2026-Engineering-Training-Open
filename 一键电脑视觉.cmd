@echo off
chcp 65001 >nul
title PC Vision - STM32 Service
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0vision\pc_vision\launch.ps1" -Mode run -NoPause
if errorlevel 1 pause
