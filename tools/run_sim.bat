@echo off
REM ===================================================================
REM  run_sim.bat - one-click entry point for the MATLAB navigation sim
REM
REM  SCOPE: this script only ever touches sim\ and sim\out\. It never runs
REM  anything under Core\ - the firmware is maintained separately.
REM
REM  Usage (double-click, or from a terminal):
REM     run_sim.bat              run the tests + the batch report
REM     run_sim.bat report       batch report only
REM     run_sim.bat test         the regression test suites
REM     run_sim.bat png          write the field / leg / mission figures to sim\out
REM     run_sim.bat scan         parameter sweep
REM     run_sim.bat stops        stop-point tolerance sweep
REM     run_sim.bat arc          arc-turn vs pivot-turn comparison
REM     run_sim.bat ring         is the outer band a legal driving lane?
REM     run_sim.bat gui          open the MATLAB DESKTOP in sim\matlab
REM                              (needed for the animation - -batch has no window)
REM
REM  Output is written to sim\out\ so you can read it afterwards. A log file
REM  is kept on failure too, which is the whole point of not deleting it.
REM ===================================================================
setlocal
set "ROOT=%~dp0.."
pushd "%ROOT%"

REM MATLAB refuses to start if its preference directory is unwritable. Point it
REM inside the project: this is the "failed to load settings plugin" fake crash.
set "MATLAB_PREFDIR=%ROOT%\tmp\mpref"
set "MATLAB=D:\matlab\bin\matlab.exe"

if not exist "%MATLAB%" (
  echo [ERROR] MATLAB not found at %MATLAB%
  echo         Edit MATLAB= in this file if it lives elsewhere.
  popd & exit /b 1
)

if "%~1"=="gui" goto :gui
if "%~1"==""     set "CMD=test_build_grid; test_kinematics; sim_report;"
if "%~1"=="report" set "CMD=sim_report;"
if "%~1"=="test"   set "CMD=test_build_grid; test_kinematics; test_zone_scripts;"
if "%~1"=="png"    set "CMD=sim_png;"
if "%~1"=="scan"   set "CMD=sim_scan;"
if "%~1"=="stops"  set "CMD=sim_scan_stops;"
if "%~1"=="arc"    set "CMD=sim_arc_vs_pivot; arc_why; arc_limit;"
if "%~1"=="ring"   set "CMD=test_ring_scope;"

REM Unknown argument MUST fail loudly. An earlier version ran whatever it was
REM given as a MATLAB command, so a quoting mistake created a stray file named
REM "qr')" in the repo root instead of reporting a bad argument.
if not defined CMD (
  echo [ERROR] unknown argument: %1
  echo.
  echo   run_sim.bat            tests + batch report
  echo   run_sim.bat report     batch report only
  echo   run_sim.bat test       regression test suites
  echo   run_sim.bat png        field / leg / mission figures into sim\out
  echo   run_sim.bat scan       parameter sweep
  echo   run_sim.bat stops      stop-point tolerance
  echo   run_sim.bat arc        arc vs pivot ^(+ why arcs are impossible^)
  echo   run_sim.bat ring       outer band: is it a legal lane?
  echo   run_sim.bat gui        MATLAB desktop ^(needed for animation^)
  echo.
  echo   To run an arbitrary MATLAB statement, use matlab -batch directly
  echo   rather than passing it here.
  popd ^& exit /b 2
)

if not exist "sim\out" mkdir "sim\out"
set "LOG=%ROOT%\sim\out\last_run.txt"

echo.
echo Running: %CMD%
echo Log will be written to: sim\out\last_run.txt
echo.
"%MATLAB%" -batch "cd('%ROOT%\sim\matlab'); %CMD%" > "%LOG%" 2>&1
set "RC=%ERRORLEVEL%"

REM MATLAB prints harmless "Unable to load ApplicationService" noise; filter it
REM so a failure is actually visible.
findstr /V /C:"Unable to load ApplicationService" "%LOG%"

echo.
if "%RC%"=="0" (
  echo [OK] finished, exit code 0
) else (
  echo [FAILED] exit code %RC% - read sim\out\last_run.txt
)
popd
exit /b %RC%

:gui
echo.
echo Opening the MATLAB desktop with the working folder set to sim\matlab.
echo In the MATLAB command window, try:
echo.
echo   P = sim_params();
echo   map = build_grid(load_mapspec());
echo   car = car_init(2250, 2250, [0 -1]);
echo   h = car_draw(map, car);
echo   out = run_leg(map, car, 'h2', 'qr', P);
echo   for i = 1:numel(out.samples)
echo       c = out.car; c.x = out.samples(i).x; c.y = out.samples(i).y; c.h = out.samples(i).h;
echo       car_draw_update(h, c); drawnow limitrate;
echo   end
echo.
"%MATLAB%" -sd "%ROOT%\sim\matlab"
popd
exit /b 0
