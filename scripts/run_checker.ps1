param(
  [Parameter(Mandatory=$true)] [string]$InstancePath,
  [Parameter(Mandatory=$true)] [string]$SolutionPath
)

$checkerDir = Join-Path $PSScriptRoot "..\checker"
$checkerExe = Join-Path $checkerDir "checker_windows.exe"

if (!(Test-Path $checkerExe)) { throw "Checker não encontrado: $checkerExe" }
if (!(Test-Path $InstancePath)) { throw "Instance não existe: $InstancePath" }
if (!(Test-Path $SolutionPath)) { throw "Solution não existe: $SolutionPath" }

# Resolvemos caminhos ABSOLUTOS ANTES de trocar de diretoria
$instAbs = (Resolve-Path -LiteralPath $InstancePath).Path
$solAbs  = (Resolve-Path -LiteralPath $SolutionPath).Path

try { Unblock-File $checkerExe } catch {}

Push-Location $checkerDir
try {
  # usar flags explícitos (checker exige --instance)
  & $checkerExe --instance $instAbs --solution $solAbs
  if ($LASTEXITCODE -ne 0) {
    # fallback 1: sem --solution
    & $checkerExe --instance $instAbs $solAbs
  }
  if ($LASTEXITCODE -ne 0) {
    # fallback 2: só --instance (pode abrir UI para escolher a solution)
    & $checkerExe --instance $instAbs
  }
  if ($LASTEXITCODE -ne 0) {
    throw "Checker terminou com código $LASTEXITCODE"
  }
}
finally { Pop-Location }