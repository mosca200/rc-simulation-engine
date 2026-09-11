param(
    [string]$OutputDirectory = "docs/validation/rv2_6_controlled_visual",
    [switch]$Force
)

$ErrorActionPreference = "Stop"

if ($env:OS -ne "Windows_NT") {
    throw "RV2-6 window capture requires Windows"
}

$branch = git branch --show-current
if ($LASTEXITCODE -ne 0 -or $branch -ne "feature/rv2-6-aerial-perspective") {
    throw "run from feature/rv2-6-aerial-perspective"
}

cargo build -p rcsim-app --release
if ($LASTEXITCODE -ne 0) {
    throw "release build failed"
}

$executable = (Resolve-Path "target/release/rcsim-app.exe").Path
$outputRoot = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $OutputDirectory))
$repositoryRoot = [System.IO.Path]::GetFullPath((Get-Location).Path)
if (-not $outputRoot.StartsWith($repositoryRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "output directory must remain inside the repository"
}
New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null

Add-Type -AssemblyName System.Drawing
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class Rv26WindowCapture {
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left; public int Top; public int Right; public int Bottom; }
    [StructLayout(LayoutKind.Sequential)]
    public struct POINT { public int X; public int Y; }
    [DllImport("user32.dll")]
    public static extern bool GetClientRect(IntPtr window, out RECT rectangle);
    [DllImport("user32.dll")]
    public static extern bool ClientToScreen(IntPtr window, ref POINT point);
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr window);
    [DllImport("user32.dll")]
    public static extern bool ShowWindowAsync(IntPtr window, int command);
    [DllImport("user32.dll")]
    public static extern bool SetWindowPos(
        IntPtr window,
        IntPtr insertAfter,
        int x,
        int y,
        int width,
        int height,
        uint flags
    );
}
"@

$cases = @(
    @{ File = "AP_ON_near.png"; Case = "near"; Ap = "on" },
    @{ File = "AP_OFF_near.png"; Case = "near"; Ap = "off" },
    @{ File = "AP_ON_100m.png"; Case = "100m"; Ap = "on" },
    @{ File = "AP_OFF_100m.png"; Case = "100m"; Ap = "off" },
    @{ File = "AP_ON_500m.png"; Case = "500m"; Ap = "on" },
    @{ File = "AP_OFF_500m.png"; Case = "500m"; Ap = "off" },
    @{ File = "AP_ON_1000m.png"; Case = "1000m"; Ap = "on" },
    @{ File = "AP_OFF_1000m.png"; Case = "1000m"; Ap = "off" },
    @{ File = "AP_ON_frontlit.png"; Case = "frontlit"; Ap = "on" },
    @{ File = "AP_OFF_frontlit.png"; Case = "frontlit"; Ap = "off" },
    @{ File = "AP_ON_sidelit.png"; Case = "sidelit"; Ap = "on" },
    @{ File = "AP_OFF_sidelit.png"; Case = "sidelit"; Ap = "off" },
    @{ File = "AP_ON_backlit.png"; Case = "backlit"; Ap = "on" },
    @{ File = "AP_OFF_backlit.png"; Case = "backlit"; Ap = "off" }
)

$expectedSize = $null
foreach ($case in $cases) {
    $outputPath = Join-Path $outputRoot $case.File
    if (Test-Path -LiteralPath $outputPath) {
        if (-not $Force) {
            throw "$outputPath already exists; use -Force to replace controlled evidence"
        }
        Remove-Item -Force -LiteralPath $outputPath
    }

    $arguments = @(
        "render",
        "--renderer", "v2",
        "--exposure-ev", "0.0",
        "--rv2-6-validation-scene", $case.Case,
        "--rv2-6-validation-ap", $case.Ap
    )
    $process = Start-Process -FilePath $executable -ArgumentList $arguments -PassThru
    try {
        $deadline = (Get-Date).AddMinutes(2)
        do {
            Start-Sleep -Milliseconds 250
            $process.Refresh()
            if ($process.HasExited) {
                throw "$($case.File): simulator exited before its window was ready"
            }
        } while ($process.MainWindowHandle -eq 0 -and (Get-Date) -lt $deadline)
        if ($process.MainWindowHandle -eq 0) {
            throw "$($case.File): simulator window was not found"
        }

        # CopyFromScreen captures whatever is actually visible. Windows may
        # reject a background process' foreground request, so keep the target
        # viewport topmost for the short capture interval.
        [Rv26WindowCapture]::ShowWindowAsync($process.MainWindowHandle, 5) | Out-Null
        [Rv26WindowCapture]::SetWindowPos(
            $process.MainWindowHandle,
            [IntPtr](-1),
            0,
            0,
            0,
            0,
            0x0043
        ) | Out-Null
        [Rv26WindowCapture]::SetForegroundWindow($process.MainWindowHandle) | Out-Null
        Start-Sleep -Seconds 6
        $process.Refresh()

        $rectangle = New-Object Rv26WindowCapture+RECT
        if (-not [Rv26WindowCapture]::GetClientRect($process.MainWindowHandle, [ref]$rectangle)) {
            throw "$($case.File): GetClientRect failed"
        }
        $origin = New-Object Rv26WindowCapture+POINT
        if (-not [Rv26WindowCapture]::ClientToScreen($process.MainWindowHandle, [ref]$origin)) {
            throw "$($case.File): ClientToScreen failed"
        }
        $width = $rectangle.Right - $rectangle.Left
        $height = $rectangle.Bottom - $rectangle.Top
        $size = "${width}x${height}"
        if ($null -eq $expectedSize) {
            $expectedSize = $size
        } elseif ($size -ne $expectedSize) {
            throw "$($case.File): viewport $size differs from $expectedSize"
        }

        $bitmap = New-Object System.Drawing.Bitmap($width, $height)
        $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
        try {
            $graphics.CopyFromScreen(
                $origin.X,
                $origin.Y,
                0,
                0,
                [System.Drawing.Size]::new($width, $height)
            )
            $bitmap.Save($outputPath, [System.Drawing.Imaging.ImageFormat]::Png)
        } finally {
            $graphics.Dispose()
            $bitmap.Dispose()
        }
        Write-Output "$($case.File): $size"
    } finally {
        if (-not $process.HasExited) {
            $process.CloseMainWindow() | Out-Null
            $process.WaitForExit(5000) | Out-Null
        }
        if (-not $process.HasExited) {
            Stop-Process -Id $process.Id -Force
        }
    }
}

Write-Output "RV2-6 controlled evidence captured at $outputRoot"
