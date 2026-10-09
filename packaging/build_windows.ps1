[CmdletBinding()]
param(
    [string]$Python = "python",
    [string]$FirmwarePackage = ""
)

$ErrorActionPreference = "Stop"

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$distRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "dist"))
$sessionName = "a2c-sensor-tools-build-$PID-$([guid]::NewGuid().ToString('N'))"
$sessionRoot = [System.IO.Path]::GetFullPath(
    (Join-Path ([System.IO.Path]::GetTempPath()) $sessionName)
)
$buildRoot = Join-Path $sessionRoot "pyinstaller"
$stageRoot = Join-Path $sessionRoot "stage"

if (-not $distRoot.StartsWith($repoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to use distribution path outside repository: $distRoot"
}
if (-not $sessionRoot.StartsWith(
    [System.IO.Path]::GetTempPath(),
    [System.StringComparison]::OrdinalIgnoreCase
)) {
    throw "Refusing to use build path outside the temporary directory: $sessionRoot"
}
New-Item -ItemType Directory -Path $buildRoot, $stageRoot, $distRoot -Force | Out-Null

$versionOutput = & $Python -c "from a2c_sensor_tools.a2c_app_support import APP_VERSION; print(APP_VERSION)"
if ($LASTEXITCODE -ne 0) {
    throw "Unable to read application version"
}
$version = $versionOutput.Trim()

$pythonExecutable = (Get-Command $Python -ErrorAction Stop).Source
$pythonDirectory = Split-Path -Parent $pythonExecutable
$controlledPath = @(
    $pythonDirectory
    (Join-Path $pythonDirectory "Scripts")
    (Join-Path $env:SystemRoot "System32")
    $env:SystemRoot
) -join ";"
$originalPath = $env:PATH

try {
    # Avoid collecting unrelated DLLs from developer tools on PATH. In
    # particular, Qt must use the Windows ICU runtime expected by PySide6.
    $env:PATH = $controlledPath

    & $pythonExecutable -m PyInstaller `
        --noconfirm `
        --clean `
        --onedir `
        --windowed `
        --contents-directory "_internal" `
        --name "A2C-Sensor-Firmware-Updater" `
        --distpath $stageRoot `
        --workpath (Join-Path $buildRoot "gui-updater") `
        --specpath $buildRoot `
        (Join-Path $repoRoot "packaging\firmware_updater_entry.py")
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware updater GUI packaging failed"
    }

    $updaterDirectory = Join-Path $stageRoot "A2C-Sensor-Firmware-Updater"
    $appDirectory = Join-Path $stageRoot "A2C-Sensor-Tools"
    Move-Item -LiteralPath $updaterDirectory -Destination $appDirectory
    $internalDirectory = Join-Path $appDirectory "_internal"

    & $pythonExecutable -m PyInstaller `
        --noconfirm `
        --clean `
        --onedir `
        --windowed `
        --contents-directory "_internal" `
        --name "A2C-IMU-Dashboard" `
        --distpath $stageRoot `
        --workpath (Join-Path $buildRoot "gui-dashboard") `
        --specpath $buildRoot `
        (Join-Path $repoRoot "packaging\dashboard_entry.py")
    if ($LASTEXITCODE -ne 0) {
        throw "IMU dashboard GUI packaging failed"
    }

    # Both applications are built with the same Python and PySide6 runtime.
    # Merge the dashboard's Qt Charts additions into the updater's _internal
    # directory so customers get two clearly named EXEs in one folder without
    # duplicating the complete runtime.
    $dashboardDirectory = Join-Path $stageRoot "A2C-IMU-Dashboard"
    $dashboardExecutable = Join-Path $dashboardDirectory "A2C-IMU-Dashboard.exe"
    $dashboardInternal = Join-Path $dashboardDirectory "_internal"
    Copy-Item -LiteralPath $dashboardExecutable -Destination $appDirectory -Force
    Copy-Item -Path (Join-Path $dashboardInternal "*") -Destination $internalDirectory -Recurse -Force

    & $pythonExecutable -m PyInstaller `
        --noconfirm `
        --clean `
        --onefile `
        --console `
        --name "a2c-firmware-update" `
        --distpath $internalDirectory `
        --workpath (Join-Path $buildRoot "cli") `
        --specpath $buildRoot `
        (Join-Path $repoRoot "packaging\firmware_update_cli_entry.py")
    if ($LASTEXITCODE -ne 0) {
        throw "Command-line helper packaging failed"
    }
} finally {
    $env:PATH = $originalPath
}

Copy-Item -LiteralPath (Join-Path $repoRoot "packaging\CUSTOMER_README.txt") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "LICENSE") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "NOTICE") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "THIRD_PARTY_NOTICES.md") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "docs\PEAK_HARDWARE_VALIDATION.md") -Destination $appDirectory

# PySide6's Qt6Core uses the Windows ICU runtime. An unrelated unversioned
# icuuc.dll collected from another developer tool shadows the Windows DLL and
# makes QtCore fail at startup with "The specified procedure could not be
# found." Refuse to publish such a package.
$unexpectedIcu = @(Get-ChildItem -LiteralPath $appDirectory -Recurse -File -Filter "icuuc.dll")
if ($unexpectedIcu.Count -ne 0) {
    $paths = $unexpectedIcu.FullName -join ", "
    throw "Unexpected ICU runtime collected in customer package: $paths"
}

if ($FirmwarePackage) {
    $firmwarePath = [System.IO.Path]::GetFullPath($FirmwarePackage)
    if (-not (Test-Path -LiteralPath $firmwarePath -PathType Leaf)) {
        throw "Firmware package does not exist: $firmwarePath"
    }
    if ([System.IO.Path]::GetExtension($firmwarePath) -ne ".binenc") {
        throw "Firmware package must use the .binenc extension"
    }
    Copy-Item -LiteralPath $firmwarePath -Destination $appDirectory
}

$cliExecutable = Join-Path $internalDirectory "a2c-firmware-update.exe"
& $cliExecutable --help | Out-Null
if ($LASTEXITCODE -ne 0) {
    throw "Packaged command-line helper smoke test failed"
}

$archiveName = "A2C-Sensor-Tools-$version-Windows-x64.zip"
$archivePath = Join-Path $distRoot $archiveName
if (Test-Path -LiteralPath $archivePath) {
    Remove-Item -LiteralPath $archivePath -Force
}
Compress-Archive -LiteralPath $appDirectory -DestinationPath $archivePath -CompressionLevel Optimal
$hash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
$hashPath = "$archivePath.sha256"
if (Test-Path -LiteralPath $hashPath) {
    Remove-Item -LiteralPath $hashPath -Force
}
Set-Content -LiteralPath $hashPath -Encoding ascii -Value "$hash  $archiveName"

try {
    Remove-Item -LiteralPath $sessionRoot -Recurse -Force
} catch {
    Write-Warning "Temporary build directory could not be removed: $sessionRoot"
}

Write-Host "Package: $archivePath"
Write-Host "SHA-256: $hash"
