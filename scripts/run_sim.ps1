$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$run = Join-Path $repo 'build/sim'
New-Item -ItemType Directory -Path $run -Force | Out-Null
Copy-Item (Join-Path $repo 'rtl/mem/*.mem') $run
Copy-Item (Join-Path $repo 'tb/mem/*.mem') $run
$rtl = @(Get-ChildItem (Join-Path $repo 'rtl/*.v') | ForEach-Object FullName)
$tb = @(Get-ChildItem (Join-Path $repo 'tb/*.sv') | ForEach-Object FullName)
Push-Location $run
try {
    & xvlog --sv @rtl @tb
    if ($LASTEXITCODE -ne 0) { throw 'xvlog failed' }
    & xelab tb_stft_istft_axi -s stft_axi_sim
    if ($LASTEXITCODE -ne 0) { throw 'xelab failed' }
    & xsim stft_axi_sim -runall
    if ($LASTEXITCODE -ne 0) { throw 'xsim failed' }
} finally { Pop-Location }
