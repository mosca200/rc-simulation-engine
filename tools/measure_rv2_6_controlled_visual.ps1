<#
.SYNOPSIS
    Measures the pixel separation between the RV2-6 controlled AP ON/OFF captures.

.DESCRIPTION
    Compares the lossless PNG pairs produced by
    tools/capture_rv2_6_controlled_visual.ps1 and reports, per pair, the number of
    changed pixels, the peak per-channel difference, the mean absolute difference
    per channel, and the direction of the change.

    A single run cannot distinguish signal from capture noise, so the tool also
    supports an ad-hoc comparison of two arbitrary images. Use it to capture the
    same configuration twice and confirm the zero-difference noise floor before
    reading the AP ON/OFF numbers.

    The bottom five rows are excluded because the window copy used for capture
    paints the rounded corners and shadow over them; that band is the only place
    where differences above one 8-bit level appear.

.PARAMETER OutputDirectory
    Evidence directory holding the AP_ON_*/AP_OFF_* pairs.

.PARAMETER Left
    Optional first image for an ad-hoc comparison.

.PARAMETER Right
    Optional second image for an ad-hoc comparison.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tools/measure_rv2_6_controlled_visual.ps1

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tools/measure_rv2_6_controlled_visual.ps1 `
        -Left tmp/on_a.png -Right tmp/on_b.png
#>
param(
    [string]$OutputDirectory = "docs/validation/rv2_6_controlled_visual",
    [string]$Left,
    [string]$Right
)

$ErrorActionPreference = "Stop"

if ($env:OS -ne "Windows_NT") {
    throw "RV2-6 capture measurement requires Windows"
}
# `-File` binds unset string parameters to "" rather than $null.
$hasLeft = -not [string]::IsNullOrWhiteSpace($Left)
$hasRight = -not [string]::IsNullOrWhiteSpace($Right)
if ($hasLeft -ne $hasRight) {
    throw "-Left and -Right must be provided together"
}

Add-Type -AssemblyName System.Drawing
Add-Type -TypeDefinition @"
using System;
public static class Rv26PairMetrics {
    // Rows are trimmed because the window copy leaves a corner/shadow band
    // there; it is never scene content.
    const int TRIMMED_ROWS = 5;

    public static string Compare(byte[] a, int strideA, byte[] b, int strideB, int width, int height) {
        int usableRows = height - TRIMMED_ROWS;
        int boxLeft = (int)(width * 0.40), boxRight = (int)(width * 0.60);
        int boxTop = (int)(height * 0.38), boxBottom = (int)(height * 0.62);

        long changed = 0, aboveOne = 0, brighterLeft = 0, brighterRight = 0;
        long absoluteSum = 0, sampleCount = 0;
        long boxChanged = 0, boxAbsoluteSum = 0, boxSamples = 0;
        int peak = 0, boxPeak = 0, peakX = -1, peakY = -1;

        for (int y = 0; y < usableRows; y++) {
            int rowA = y * strideA, rowB = y * strideB;
            bool insideBoxRow = y >= boxTop && y < boxBottom;
            for (int x = 0; x < width; x++) {
                int i = rowA + x * 3, j = rowB + x * 3;
                int d0 = a[i] - b[j];
                int d1 = a[i + 1] - b[j + 1];
                int d2 = a[i + 2] - b[j + 2];
                int channelPeak = Math.Max(Math.Abs(d0), Math.Max(Math.Abs(d1), Math.Abs(d2)));
                int channelSum = Math.Abs(d0) + Math.Abs(d1) + Math.Abs(d2);

                absoluteSum += channelSum;
                sampleCount += 3;
                if (channelPeak > 0) {
                    changed++;
                    int signedSum = d0 + d1 + d2;
                    if (signedSum > 0) brighterLeft++;
                    else if (signedSum < 0) brighterRight++;
                }
                if (channelPeak > 1) aboveOne++;
                if (channelPeak > peak) {
                    peak = channelPeak;
                    peakX = x;
                    peakY = y;
                }
                if (insideBoxRow && x >= boxLeft && x < boxRight) {
                    boxAbsoluteSum += channelSum;
                    boxSamples += 3;
                    if (channelPeak > 0) boxChanged++;
                    if (channelPeak > boxPeak) boxPeak = channelPeak;
                }
            }
        }

        double visiblePixels = (double)usableRows * width;
        return string.Format(
            "changedPx={0} ({1:F3}%) aboveOneLevelPx={2} peakChannelDiff={3}@{4},{5} meanAbsPerChannel={6:F4} " +
            "leftBrighterPx={7} rightBrighterPx={8} | targetBox: changedPx={9} meanAbsPerChannel={10:F4} peakChannelDiff={11}",
            changed, 100.0 * changed / visiblePixels, aboveOne, peak, peakX, peakY,
            sampleCount > 0 ? (double)absoluteSum / sampleCount : 0.0,
            brighterLeft, brighterRight,
            boxChanged, boxSamples > 0 ? (double)boxAbsoluteSum / boxSamples : 0.0, boxPeak);
    }
}
"@

function Read-Rgb([string]$path) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "missing input image: $path"
    }
    $bitmap = [System.Drawing.Bitmap]::FromFile([System.IO.Path]::GetFullPath($path))
    try {
        $data = $bitmap.LockBits(
            [System.Drawing.Rectangle]::new(0, 0, $bitmap.Width, $bitmap.Height),
            [System.Drawing.Imaging.ImageLockMode]::ReadOnly,
            [System.Drawing.Imaging.PixelFormat]::Format24bppRgb
        )
        try {
            $stride = $data.Stride
            $bytes = New-Object byte[] ($stride * $bitmap.Height)
            [System.Runtime.InteropServices.Marshal]::Copy($data.Scan0, $bytes, 0, $bytes.Length)
        } finally {
            $bitmap.UnlockBits($data)
        }
        return [pscustomobject]@{
            Width = $bitmap.Width; Height = $bitmap.Height; Stride = $stride; Bytes = $bytes
        }
    } finally {
        $bitmap.Dispose()
    }
}

if ($hasLeft) {
    $leftImage = Read-Rgb $Left
    $rightImage = Read-Rgb $Right
    if ($leftImage.Width -ne $rightImage.Width -or $leftImage.Height -ne $rightImage.Height) {
        throw "image sizes differ: $($leftImage.Width)x$($leftImage.Height) vs $($rightImage.Width)x$($rightImage.Height)"
    }
    "$Left vs $Right : " + [Rv26PairMetrics]::Compare(
        $leftImage.Bytes, $leftImage.Stride,
        $rightImage.Bytes, $rightImage.Stride,
        $leftImage.Width, $leftImage.Height
    )
    return
}

foreach ($case in @("near", "100m", "500m", "1000m", "frontlit", "sidelit", "backlit")) {
    $leftImage = Read-Rgb (Join-Path $OutputDirectory "AP_ON_${case}.png")
    $rightImage = Read-Rgb (Join-Path $OutputDirectory "AP_OFF_${case}.png")
    if ($leftImage.Width -ne $rightImage.Width -or $leftImage.Height -ne $rightImage.Height) {
        throw "$case : image sizes differ"
    }
    "${case}: " + [Rv26PairMetrics]::Compare(
        $leftImage.Bytes, $leftImage.Stride,
        $rightImage.Bytes, $rightImage.Stride,
        $leftImage.Width, $leftImage.Height
    )
}
