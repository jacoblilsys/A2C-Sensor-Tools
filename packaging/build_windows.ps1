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

& $Python -m PyInstaller `
    --noconfirm `
    --clean `
    --onedir `
    --windowed `
    --contents-directory "_internal" `
    --name "A2C-Sensor-Firmware-Updater" `
    --distpath $stageRoot `
    --workpath (Join-Path $buildRoot "gui") `
    --specpath $buildRoot `
    (Join-Path $repoRoot "packaging\firmware_updater_entry.py")
if ($LASTEXITCODE -ne 0) {
    throw "GUI packaging failed"
}

$appDirectory = Join-Path $stageRoot "A2C-Sensor-Firmware-Updater"
& $Python -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --console `
    --name "a2c-firmware-update" `
    --distpath $appDirectory `
    --workpath (Join-Path $buildRoot "cli") `
    --specpath $buildRoot `
    (Join-Path $repoRoot "packaging\firmware_update_cli_entry.py")
if ($LASTEXITCODE -ne 0) {
    throw "Command-line helper packaging failed"
}

Copy-Item -LiteralPath (Join-Path $repoRoot "packaging\CUSTOMER_README.txt") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "LICENSE") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "NOTICE") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "THIRD_PARTY_NOTICES.md") -Destination $appDirectory
Copy-Item -LiteralPath (Join-Path $repoRoot "docs\PEAK_HARDWARE_VALIDATION.md") -Destination $appDirectory

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

$cliExecutable = Join-Path $appDirectory "a2c-firmware-update.exe"
& $cliExecutable --help | Out-Null
if ($LASTEXITCODE -ne 0) {
    throw "Packaged command-line helper smoke test failed"
}

$archiveName = "A2C-Sensor-Tools-$version-Windows-x64-PEAK-test.zip"
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
