[CmdletBinding()]
param(
    [ValidateRange(1, 100)][int]$Rate = 20,
    [ValidateRange(1024, 65535)][int]$Port = 8765,
    [string]$PythonPath = ''
)

$ErrorActionPreference = 'Stop'
$projectDirectory = $PSScriptRoot
$venvPython = Join-Path $projectDirectory '.venv\Scripts\python.exe'

if (-not (Test-Path -LiteralPath $venvPython -PathType Leaf)) {
    if ($PythonPath) {
        if (-not (Test-Path -LiteralPath $PythonPath -PathType Leaf)) {
            throw "Python executable not found: $PythonPath"
        }
        & $PythonPath -m venv (Join-Path $projectDirectory '.venv')
    } else {
        $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
        $pyCommand = Get-Command py -ErrorAction SilentlyContinue
        if ($pyCommand) {
            & $pyCommand.Source -3 -m venv (Join-Path $projectDirectory '.venv')
        } elseif ($pythonCommand) {
            & $pythonCommand.Source -m venv (Join-Path $projectDirectory '.venv')
        } else {
            throw 'Install Python 3.10 or later, or pass -PythonPath with the full path to python.exe.'
        }
    }
    if ($LASTEXITCODE -ne 0) { throw 'Could not create the local Python environment.' }
}

# Install only when a required dependency is missing or the ODrive pin differs.
& $venvPython -c 'import importlib.metadata; import numpy; assert importlib.metadata.version("odrive") == "0.6.11.post1"' 2>$null
if ($LASTEXITCODE -ne 0) {
    & $venvPython -m pip install -r (Join-Path $projectDirectory 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed. Check the network connection and retry.' }
}

Write-Host "Open http://127.0.0.1:$Port . Starting the server does not connect or start either drive."
& $venvPython (Join-Path $projectDirectory 'app.py') --port $Port --rate $Rate
if ($LASTEXITCODE -ne 0) { throw "The dashboard stopped with exit code $LASTEXITCODE." }
