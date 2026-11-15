param(
    [ValidateSet('preview','run','check')][string]$Mode = 'preview',
    [switch]$PrepareOnly,
    [switch]$NoPause
)
$ErrorActionPreference = 'Stop'
$scriptRoot = $PSScriptRoot
$env:PYTHONUTF8 = '1'
$env:PYTHONUNBUFFERED = '1'

function Test-Python([string]$Candidate) {
    if (-not $Candidate -or -not (Test-Path -LiteralPath $Candidate)) { return $false }
    try {
        & $Candidate -c 'import sys; sys.exit(0 if (3,10)<=sys.version_info[:2]<(3,14) else 1)' 2>$null
        return $LASTEXITCODE -eq 0
    } catch { return $false }
}

try {
    $runtimeDir = Join-Path $env:LOCALAPPDATA 'EngineeringTrainingPcVision\venv'
    $runtimePython = Join-Path $runtimeDir 'Scripts\python.exe'
    if (-not (Test-Python $runtimePython)) {
        $candidatePython = $null
        $pyLauncher = Get-Command py -ErrorAction SilentlyContinue
        if ($pyLauncher) {
            foreach ($version in @('-3.12','-3.13','-3.11','-3.10')) {
                try {
                    $resolvedPython = & $pyLauncher.Source $version -c 'import sys; print(sys.executable)' 2>$null
                    if ($LASTEXITCODE -eq 0 -and (Test-Python $resolvedPython)) { $candidatePython = $resolvedPython; break }
                } catch {}
            }
        }
        if (-not $candidatePython) {
            $systemPython = Get-Command python -ErrorAction SilentlyContinue
            if ($systemPython -and (Test-Python $systemPython.Source)) { $candidatePython = $systemPython.Source }
        }
        if (-not $candidatePython) {
            $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
            if (Test-Python $bundledPython) { $candidatePython = $bundledPython }
        }
        if (-not $candidatePython) {
            throw 'Python 3.10-3.13 was not found. Install Python 3.12 from https://www.python.org/downloads/windows/ then run again.'
        }
        Write-Host 'Preparing a private Python environment (first run only)...'
        $ErrorActionPreference = 'Continue'
        & $candidatePython -m venv $runtimeDir
        $ErrorActionPreference = 'Stop'
        if ($LASTEXITCODE -ne 0) { throw 'Could not create Python environment.' }
    }
    $ErrorActionPreference = 'Continue'
    & $runtimePython -c "import importlib.util as u,sys; sys.exit(0 if u.find_spec('cv2') and u.find_spec('numpy') and u.find_spec('serial') else 1)" 2>$null
    $dependencyStatus = $LASTEXITCODE
    if ($dependencyStatus -eq 0) {
        & $runtimePython -c 'import cv2,numpy; assert int(cv2.__version__[0]) == 4' 2>$null
        $dependencyStatus = $LASTEXITCODE
    }
    $ErrorActionPreference = 'Stop'
    if ($dependencyStatus -ne 0) {
        Write-Host 'Installing OpenCV, NumPy and pySerial (first run requires internet)...'
        $ErrorActionPreference = 'Continue'
        & $runtimePython -m pip install --disable-pip-version-check -r (Join-Path $scriptRoot 'requirements.txt')
        $ErrorActionPreference = 'Stop'
        if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed. Check internet and retry; details are shown above.' }
    }
    $ErrorActionPreference = 'Continue'
    if ($PrepareOnly -or $Mode -eq 'check') {
        & $runtimePython (Join-Path $scriptRoot 'main.py') --check
    } else {
        Write-Host ('Starting PC vision desktop panel: ' + $Mode)
        & $runtimePython (Join-Path $scriptRoot 'gui.py') --mode $Mode
    }
    $ErrorActionPreference = 'Stop'
    if ($LASTEXITCODE -ne 0) { throw 'PC vision exited with an error; see the output above.' }
} catch {
    Write-Host $_ -ForegroundColor Red
    if (-not $NoPause) { [void](Read-Host 'Press Enter to close') }
    exit 1
}
