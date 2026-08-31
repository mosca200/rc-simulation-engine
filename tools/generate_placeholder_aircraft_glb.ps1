param(
    [string]$OutputPath = "models/acro_electric_01/aircraft.glb"
)

$positions = [System.Collections.Generic.List[float]]::new()
$colors = [System.Collections.Generic.List[float]]::new()
$indices = [System.Collections.Generic.List[uint32]]::new()

function Add-Box {
    param([float[]]$Minimum, [float[]]$Maximum, [float[]]$Color)
    $base = [uint32]($positions.Count / 3)
    $corners = @(
        @($Minimum[0], $Minimum[1], $Minimum[2]), @($Minimum[0], $Maximum[1], $Minimum[2]),
        @($Minimum[0], $Minimum[1], $Maximum[2]), @($Minimum[0], $Maximum[1], $Maximum[2]),
        @($Maximum[0], $Minimum[1], $Minimum[2]), @($Maximum[0], $Maximum[1], $Minimum[2]),
        @($Maximum[0], $Minimum[1], $Maximum[2]), @($Maximum[0], $Maximum[1], $Maximum[2])
    )
    foreach ($corner in $corners) {
        foreach ($value in $corner) { $positions.Add([float]$value) }
        foreach ($value in $Color) { $colors.Add([float]$value) }
    }
    $boxIndices = @(4,5,7, 4,7,6, 2,3,1, 2,1,0, 1,3,7, 1,7,5,
                    2,0,4, 2,4,6, 2,6,7, 2,7,3, 4,0,1, 4,1,5)
    foreach ($index in $boxIndices) { $indices.Add($base + [uint32]$index) }
}

# Presentation placeholder in render-local coordinates: +X right, +Y up, -Z nose.
Add-Box @(-0.13,-0.12,-1.10) @(0.13,0.12,1.00) @(0.16,0.31,0.72)
Add-Box @(-0.16,-0.14,-1.50) @(0.16,0.14,-1.10) @(0.94,0.22,0.08)
Add-Box @(-1.35,-0.04,-0.25) @(1.35,0.04,0.30) @(0.91,0.70,0.12)
Add-Box @(-0.55,-0.03,0.75) @(0.55,0.03,1.10) @(0.82,0.28,0.54)
Add-Box @(-0.04,0.08,0.65) @(0.04,0.60,1.05) @(0.18,0.72,0.25)
Add-Box @(-0.11,0.10,-0.55) @(0.11,0.23,-0.10) @(0.95,0.48,0.08)

$binaryStream = [System.IO.MemoryStream]::new()
$binaryWriter = [System.IO.BinaryWriter]::new($binaryStream)
foreach ($value in $positions) { $binaryWriter.Write([float]$value) }
$colorOffset = [int]$binaryStream.Position
foreach ($value in $colors) { $binaryWriter.Write([float]$value) }
$indexOffset = [int]$binaryStream.Position
foreach ($value in $indices) { $binaryWriter.Write([uint32]$value) }
while (($binaryStream.Length % 4) -ne 0) { $binaryWriter.Write([byte]0) }
$binary = $binaryStream.ToArray()

$vertexCount = [int]($positions.Count / 3)
$indexCount = $indices.Count
$positionLength = $positions.Count * 4
$colorLength = $colors.Count * 4
$indexLength = $indices.Count * 4
$jsonObject = [ordered]@{
    asset = [ordered]@{ version = "2.0"; generator = "RC Simulation Engine P1 placeholder generator" }
    scene = 0
    scenes = @([ordered]@{ nodes = @(0) })
    nodes = @([ordered]@{ mesh = 0; name = "AcroElectricPresentationPlaceholder" })
    meshes = @([ordered]@{ name = "AircraftPlaceholder"; primitives = @([ordered]@{
        attributes = [ordered]@{ POSITION = 0; COLOR_0 = 1 }; indices = 2; mode = 4
    }) })
    buffers = @([ordered]@{ byteLength = $binary.Length })
    bufferViews = @(
        [ordered]@{ buffer = 0; byteOffset = 0; byteLength = $positionLength; target = 34962 },
        [ordered]@{ buffer = 0; byteOffset = $colorOffset; byteLength = $colorLength; target = 34962 },
        [ordered]@{ buffer = 0; byteOffset = $indexOffset; byteLength = $indexLength; target = 34963 }
    )
    accessors = @(
        [ordered]@{ bufferView = 0; componentType = 5126; count = $vertexCount; type = "VEC3"; min = @(-1.35,-0.14,-1.50); max = @(1.35,0.60,1.10) },
        [ordered]@{ bufferView = 1; componentType = 5126; count = $vertexCount; type = "VEC3" },
        [ordered]@{ bufferView = 2; componentType = 5125; count = $indexCount; type = "SCALAR" }
    )
}
$json = $jsonObject | ConvertTo-Json -Depth 12 -Compress
$jsonBytes = [System.Text.Encoding]::UTF8.GetBytes($json)
$jsonPadding = (4 - ($jsonBytes.Length % 4)) % 4
$totalLength = 12 + 8 + $jsonBytes.Length + $jsonPadding + 8 + $binary.Length

$outputDirectory = Split-Path -Parent $OutputPath
if ($outputDirectory) { [System.IO.Directory]::CreateDirectory($outputDirectory) | Out-Null }
$output = [System.IO.File]::Open($OutputPath, [System.IO.FileMode]::Create)
$writer = [System.IO.BinaryWriter]::new($output)
$writer.Write([uint32]0x46546C67)
$writer.Write([uint32]2)
$writer.Write([uint32]$totalLength)
$writer.Write([uint32]($jsonBytes.Length + $jsonPadding))
$writer.Write([uint32]0x4E4F534A)
$writer.Write($jsonBytes)
for ($i = 0; $i -lt $jsonPadding; $i++) { $writer.Write([byte]0x20) }
$writer.Write([uint32]$binary.Length)
$writer.Write([uint32]0x004E4942)
$writer.Write($binary)
$writer.Dispose()
$binaryWriter.Dispose()

Write-Output "Generated $OutputPath ($totalLength bytes)"
