$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath (Join-Path $PSScriptRoot 'gui')
& (Join-Path $PSScriptRoot '.venv\Scripts\python.exe') -m surface_editor --ai @args
exit $LASTEXITCODE
