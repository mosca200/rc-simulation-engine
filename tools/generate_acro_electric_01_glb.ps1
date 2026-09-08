param(
    [string]$OutputPath = "models/acro_electric_01/aircraft.glb"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$VisualYOffset = 0.255

# Deterministic, dependency-free authoring source for the G3C-B visual asset.
# Render-local coordinates are +X right, +Y up, -Z forward/nose. The production
# loader intentionally ignores glTF node transforms, so every position is baked.

function New-Mesh {
    param([string]$Name, [int]$Material)
    [pscustomobject]@{
        Name = $Name
        Material = $Material
        Positions = [System.Collections.Generic.List[float]]::new()
        Normals = [System.Collections.Generic.List[float]]::new()
        Indices = [System.Collections.Generic.List[uint32]]::new()
    }
}

function Normalize-Vector {
    param([double[]]$Vector)
    $length = [Math]::Sqrt($Vector[0] * $Vector[0] + $Vector[1] * $Vector[1] + $Vector[2] * $Vector[2])
    if ($length -le 1.0e-12) { return [double[]]@(0.0, 1.0, 0.0) }
    return [double[]]@(($Vector[0] / $length), ($Vector[1] / $length), ($Vector[2] / $length))
}

function Cross-Vector {
    param([double[]]$A, [double[]]$B)
    return [double[]]@(
        ($A[1] * $B[2] - $A[2] * $B[1]),
        ($A[2] * $B[0] - $A[0] * $B[2]),
        ($A[0] * $B[1] - $A[1] * $B[0])
    )
}

function Add-Vertex {
    param($Mesh, [double[]]$Position, [double[]]$Normal)
    $normalized = Normalize-Vector $Normal
    $index = [uint32]($Mesh.Positions.Count / 3)
    $Mesh.Positions.Add([float]$Position[0])
    $Mesh.Positions.Add([float]($Position[1] + $VisualYOffset))
    $Mesh.Positions.Add([float]$Position[2])
    foreach ($value in $normalized) { $Mesh.Normals.Add([float]$value) }
    return $index
}

function Get-VertexPosition {
    param($Mesh, [uint32]$Index)
    $offset = [int]$Index * 3
    return [double[]]@(
        $Mesh.Positions[$offset],
        $Mesh.Positions[$offset + 1],
        $Mesh.Positions[$offset + 2]
    )
}

function Get-VertexNormal {
    param($Mesh, [uint32]$Index)
    $offset = [int]$Index * 3
    return [double[]]@(
        $Mesh.Normals[$offset],
        $Mesh.Normals[$offset + 1],
        $Mesh.Normals[$offset + 2]
    )
}

function Add-TriangleFacing {
    param($Mesh, [uint32]$A, [uint32]$B, [uint32]$C)
    $pa = Get-VertexPosition $Mesh $A
    $pb = Get-VertexPosition $Mesh $B
    $pc = Get-VertexPosition $Mesh $C
    $ab = [double[]]@(($pb[0] - $pa[0]), ($pb[1] - $pa[1]), ($pb[2] - $pa[2]))
    $ac = [double[]]@(($pc[0] - $pa[0]), ($pc[1] - $pa[1]), ($pc[2] - $pa[2]))
    $face = Cross-Vector $ab $ac
    $na = Get-VertexNormal $Mesh $A
    $nb = Get-VertexNormal $Mesh $B
    $nc = Get-VertexNormal $Mesh $C
    $expected = [double[]]@(($na[0] + $nb[0] + $nc[0]), ($na[1] + $nb[1] + $nc[1]), ($na[2] + $nb[2] + $nc[2]))
    $dot = $face[0] * $expected[0] + $face[1] * $expected[1] + $face[2] * $expected[2]
    $Mesh.Indices.Add($A)
    if ($dot -ge 0.0) {
        $Mesh.Indices.Add($B); $Mesh.Indices.Add($C)
    } else {
        $Mesh.Indices.Add($C); $Mesh.Indices.Add($B)
    }
}

function Add-QuadFacing {
    param($Mesh, [uint32]$A, [uint32]$B, [uint32]$C, [uint32]$D)
    Add-TriangleFacing $Mesh $A $B $C
    Add-TriangleFacing $Mesh $A $C $D
}

function Add-LoftZ {
    param($Mesh, [object[]]$Sections, [int]$Segments = 24)
    $rings = @()
    for ($sectionIndex = 0; $sectionIndex -lt $Sections.Count; $sectionIndex++) {
        $section = $Sections[$sectionIndex]
        $ring = @()
        $previous = $Sections[[Math]::Max(0, $sectionIndex - 1)]
        $next = $Sections[[Math]::Min($Sections.Count - 1, $sectionIndex + 1)]
        $dz = [double]$next[0] - [double]$previous[0]
        $drx = [double]$next[2] - [double]$previous[2]
        $dry = [double]$next[3] - [double]$previous[3]
        for ($segment = 0; $segment -lt $Segments; $segment++) {
            $angle = 2.0 * [Math]::PI * $segment / $Segments
            $cos = [Math]::Cos($angle); $sin = [Math]::Sin($angle)
            $position = [double[]]@(($section[2] * $cos), ($section[1] + $section[3] * $sin), $section[0])
            $nz = if ([Math]::Abs($dz) -gt 1.0e-12) {
                -($drx * $cos * $cos + $dry * $sin * $sin) / $dz
            } else { 0.0 }
            $normal = [double[]]@(($cos / $section[2]), ($sin / $section[3]), $nz)
            $ring += Add-Vertex $Mesh $position $normal
        }
        $rings += ,$ring
    }
    for ($section = 0; $section -lt $rings.Count - 1; $section++) {
        for ($segment = 0; $segment -lt $Segments; $segment++) {
            $nextSegment = ($segment + 1) % $Segments
            Add-QuadFacing $Mesh $rings[$section][$segment] $rings[$section + 1][$segment] $rings[$section + 1][$nextSegment] $rings[$section][$nextSegment]
        }
    }
    foreach ($cap in @(@(0, -1.0), @((($Sections.Count - 1)), 1.0))) {
        $sectionIndex = [int]$cap[0]; $direction = [double]$cap[1]
        $sectionData = $Sections[$sectionIndex]
        $normal = [double[]]@(0.0, 0.0, $direction)
        $center = Add-Vertex $Mesh ([double[]]@(0.0, $sectionData[1], $sectionData[0])) $normal
        $capRing = @()
        for ($segment = 0; $segment -lt $Segments; $segment++) {
            $angle = 2.0 * [Math]::PI * $segment / $Segments
            $capRing += Add-Vertex $Mesh ([double[]]@(
                ($sectionData[2] * [Math]::Cos($angle)),
                ($sectionData[1] + $sectionData[3] * [Math]::Sin($angle)),
                $sectionData[0]
            )) $normal
        }
        for ($segment = 0; $segment -lt $Segments; $segment++) {
            Add-TriangleFacing $Mesh $center $capRing[$segment] $capRing[($segment + 1) % $Segments]
        }
    }
}

function Add-AirfoilPanel {
    param($Mesh, [object[]]$Stations, [ValidateSet("horizontal", "vertical")][string]$Plane = "horizontal")
    # Clockwise section from leading edge over the upper surface to trailing edge.
    $profile = @(
        @(0.000, 0.000), @(0.020, 0.180), @(0.055, 0.315), @(0.110, 0.410),
        @(0.200, 0.480), @(0.340, 0.500), @(0.500, 0.455), @(0.660, 0.360),
        @(0.800, 0.235), @(0.910, 0.115), @(1.000, 0.000), @(0.900, -0.070),
        @(0.730, -0.135), @(0.520, -0.185), @(0.310, -0.205), @(0.140, -0.155),
        @(0.045, -0.075)
    )
    $rings = @()
    foreach ($station in $Stations) {
        $span = [double]$station[0]; $center = [double]$station[1]
        $leading = [double]$station[2]; $trailing = [double]$station[3]; $thickness = [double]$station[4]
        $chord = $trailing - $leading
        $ring = @()
        for ($pointIndex = 0; $pointIndex -lt $profile.Count; $pointIndex++) {
            $point = $profile[$pointIndex]
            $previous = $profile[($pointIndex + $profile.Count - 1) % $profile.Count]
            $next = $profile[($pointIndex + 1) % $profile.Count]
            $z = $leading + $point[0] * $chord
            $thicknessPosition = $center + $point[1] * $thickness
            $dz = ($next[0] - $previous[0]) * $chord
            $dt = ($next[1] - $previous[1]) * $thickness
            if ($Plane -eq "horizontal") {
                $position = [double[]]@($span, $thicknessPosition, $z)
                $normal = [double[]]@(0.0, $dz, -$dt)
            } else {
                $position = [double[]]@($thicknessPosition, $span, $z)
                $normal = [double[]]@($dz, 0.0, -$dt)
            }
            $ring += Add-Vertex $Mesh $position $normal
        }
        $rings += ,$ring
    }
    for ($station = 0; $station -lt $rings.Count - 1; $station++) {
        for ($point = 0; $point -lt $profile.Count; $point++) {
            $nextPoint = ($point + 1) % $profile.Count
            Add-QuadFacing $Mesh $rings[$station][$point] $rings[$station + 1][$point] $rings[$station + 1][$nextPoint] $rings[$station][$nextPoint]
        }
    }
    foreach ($capIndex in @(0, ($Stations.Count - 1))) {
        $other = if ($capIndex -eq 0) { 1 } else { $Stations.Count - 2 }
        $direction = [Math]::Sign([double]$Stations[$capIndex][0] - [double]$Stations[$other][0])
        $normal = if ($Plane -eq "horizontal") { [double[]]@($direction, 0.0, 0.0) } else { [double[]]@(0.0, $direction, 0.0) }
        $centerPosition = if ($Plane -eq "horizontal") {
            [double[]]@($Stations[$capIndex][0], $Stations[$capIndex][1], (($Stations[$capIndex][2] + $Stations[$capIndex][3]) * 0.5))
        } else {
            [double[]]@($Stations[$capIndex][1], $Stations[$capIndex][0], (($Stations[$capIndex][2] + $Stations[$capIndex][3]) * 0.5))
        }
        $centerVertex = Add-Vertex $Mesh $centerPosition $normal
        $capRing = @()
        foreach ($point in $profile) {
            $z = $Stations[$capIndex][2] + $point[0] * ($Stations[$capIndex][3] - $Stations[$capIndex][2])
            $offset = $Stations[$capIndex][1] + $point[1] * $Stations[$capIndex][4]
            $position = if ($Plane -eq "horizontal") { [double[]]@($Stations[$capIndex][0], $offset, $z) } else { [double[]]@($offset, $Stations[$capIndex][0], $z) }
            $capRing += Add-Vertex $Mesh $position $normal
        }
        for ($point = 0; $point -lt $profile.Count; $point++) {
            Add-TriangleFacing $Mesh $centerVertex $capRing[$point] $capRing[($point + 1) % $profile.Count]
        }
    }
}

function Add-Cylinder {
    param($Mesh, [double[]]$Start, [double[]]$End, [double]$Radius, [int]$Segments = 12)
    $axis = Normalize-Vector ([double[]]@(($End[0] - $Start[0]), ($End[1] - $Start[1]), ($End[2] - $Start[2])))
    $reference = if ([Math]::Abs($axis[1]) -lt 0.85) { [double[]]@(0.0, 1.0, 0.0) } else { [double[]]@(1.0, 0.0, 0.0) }
    $u = Normalize-Vector (Cross-Vector $axis $reference)
    $v = Normalize-Vector (Cross-Vector $axis $u)
    $rings = @(@(), @())
    for ($endIndex = 0; $endIndex -lt 2; $endIndex++) {
        $center = if ($endIndex -eq 0) { $Start } else { $End }
        for ($segment = 0; $segment -lt $Segments; $segment++) {
            $angle = 2.0 * [Math]::PI * $segment / $Segments
            $normal = [double[]]@(
                ($u[0] * [Math]::Cos($angle) + $v[0] * [Math]::Sin($angle)),
                ($u[1] * [Math]::Cos($angle) + $v[1] * [Math]::Sin($angle)),
                ($u[2] * [Math]::Cos($angle) + $v[2] * [Math]::Sin($angle))
            )
            $position = [double[]]@(($center[0] + $Radius * $normal[0]), ($center[1] + $Radius * $normal[1]), ($center[2] + $Radius * $normal[2]))
            $rings[$endIndex] += Add-Vertex $Mesh $position $normal
        }
    }
    for ($segment = 0; $segment -lt $Segments; $segment++) {
        $next = ($segment + 1) % $Segments
        Add-QuadFacing $Mesh $rings[0][$segment] $rings[1][$segment] $rings[1][$next] $rings[0][$next]
    }
    foreach ($cap in @(@(0, -1.0), @(1, 1.0))) {
        $endIndex = [int]$cap[0]; $direction = [double]$cap[1]
        $normal = [double[]]@(($axis[0] * $direction), ($axis[1] * $direction), ($axis[2] * $direction))
        $centerPosition = if ($endIndex -eq 0) { $Start } else { $End }
        $centerVertex = Add-Vertex $Mesh $centerPosition $normal
        $capRing = @()
        for ($segment = 0; $segment -lt $Segments; $segment++) {
            $angle = 2.0 * [Math]::PI * $segment / $Segments
            $radial = [double[]]@(
                ($u[0] * [Math]::Cos($angle) + $v[0] * [Math]::Sin($angle)),
                ($u[1] * [Math]::Cos($angle) + $v[1] * [Math]::Sin($angle)),
                ($u[2] * [Math]::Cos($angle) + $v[2] * [Math]::Sin($angle))
            )
            $capRing += Add-Vertex $Mesh ([double[]]@(($centerPosition[0] + $Radius * $radial[0]), ($centerPosition[1] + $Radius * $radial[1]), ($centerPosition[2] + $Radius * $radial[2]))) $normal
        }
        for ($segment = 0; $segment -lt $Segments; $segment++) {
            Add-TriangleFacing $Mesh $centerVertex $capRing[$segment] $capRing[($segment + 1) % $Segments]
        }
    }
}

function Add-ExtrudedPolygonXY {
    param($Mesh, [object[]]$Points, [double]$FrontZ, [double]$BackZ)
    $front = @(); $back = @()
    foreach ($point in $Points) {
        $front += Add-Vertex $Mesh ([double[]]@($point[0], $point[1], $FrontZ)) ([double[]]@(0.0, 0.0, -1.0))
        $back += Add-Vertex $Mesh ([double[]]@($point[0], $point[1], $BackZ)) ([double[]]@(0.0, 0.0, 1.0))
    }
    for ($point = 1; $point -lt $Points.Count - 1; $point++) {
        Add-TriangleFacing $Mesh $front[0] $front[$point] $front[$point + 1]
        Add-TriangleFacing $Mesh $back[0] $back[$point] $back[$point + 1]
    }
    for ($point = 0; $point -lt $Points.Count; $point++) {
        $next = ($point + 1) % $Points.Count
        $edgeX = [double]$Points[$next][0] - [double]$Points[$point][0]
        $edgeY = [double]$Points[$next][1] - [double]$Points[$point][1]
        $normal = Normalize-Vector ([double[]]@($edgeY, -$edgeX, 0.0))
        $a = Add-Vertex $Mesh ([double[]]@($Points[$point][0], $Points[$point][1], $FrontZ)) $normal
        $b = Add-Vertex $Mesh ([double[]]@($Points[$next][0], $Points[$next][1], $FrontZ)) $normal
        $c = Add-Vertex $Mesh ([double[]]@($Points[$next][0], $Points[$next][1], $BackZ)) $normal
        $d = Add-Vertex $Mesh ([double[]]@($Points[$point][0], $Points[$point][1], $BackZ)) $normal
        Add-QuadFacing $Mesh $a $b $c $d
    }
}

function Add-TwistedPropellerBlade {
    param($Mesh, [double]$Direction)
    # radius, half chord and visible pitch depth. The opposite blade is the
    # exact 180-degree rotation of the first around the propeller shaft.
    $stations = @(
        @(0.060, 0.030, 0.004), @(0.105, 0.046, 0.010),
        @(0.190, 0.052, 0.016), @(0.285, 0.043, 0.019),
        @(0.350, 0.024, 0.014), @(0.370, 0.009, 0.006)
    )
    $frontLeft = @(); $frontRight = @(); $backLeft = @(); $backRight = @()
    foreach ($station in $stations) {
        $radius = [double]$station[0]; $halfChord = [double]$station[1]; $pitch = [double]$station[2]
        $centerY = $Direction * $radius
        $leftX = -$Direction * $halfChord; $rightX = $Direction * $halfChord
        $leftZ = -0.694 - $pitch; $rightZ = -0.694 + $pitch
        $frontLeft += Add-Vertex $Mesh ([double[]]@($leftX, $centerY, ($leftZ - 0.004))) ([double[]]@(0.0, 0.0, -1.0))
        $frontRight += Add-Vertex $Mesh ([double[]]@($rightX, $centerY, ($rightZ - 0.004))) ([double[]]@(0.0, 0.0, -1.0))
        $backLeft += Add-Vertex $Mesh ([double[]]@($leftX, $centerY, ($leftZ + 0.004))) ([double[]]@(0.0, 0.0, 1.0))
        $backRight += Add-Vertex $Mesh ([double[]]@($rightX, $centerY, ($rightZ + 0.004))) ([double[]]@(0.0, 0.0, 1.0))
    }
    for ($station = 0; $station -lt $stations.Count - 1; $station++) {
        $next = $station + 1
        Add-QuadFacing $Mesh $frontLeft[$station] $frontLeft[$next] $frontRight[$next] $frontRight[$station]
        Add-QuadFacing $Mesh $backLeft[$station] $backRight[$station] $backRight[$next] $backLeft[$next]
        Add-QuadFacing $Mesh $frontLeft[$station] $backLeft[$station] $backLeft[$next] $frontLeft[$next]
        Add-QuadFacing $Mesh $frontRight[$station] $frontRight[$next] $backRight[$next] $backRight[$station]
    }
    Add-QuadFacing $Mesh $frontLeft[0] $frontRight[0] $backRight[0] $backLeft[0]
    $tip = $stations.Count - 1
    Add-QuadFacing $Mesh $frontLeft[$tip] $backLeft[$tip] $backRight[$tip] $frontRight[$tip]
}

function Add-TorusX {
    param($Mesh, [double[]]$Center, [double]$MajorRadius, [double]$MinorRadius, [int]$MajorSegments = 18, [int]$MinorSegments = 8)
    $rings = @()
    for ($major = 0; $major -lt $MajorSegments; $major++) {
        $u = 2.0 * [Math]::PI * $major / $MajorSegments
        $ring = @()
        for ($minor = 0; $minor -lt $MinorSegments; $minor++) {
            $v = 2.0 * [Math]::PI * $minor / $MinorSegments
            $radial = $MajorRadius + $MinorRadius * [Math]::Cos($v)
            $position = [double[]]@(
                ($Center[0] + $MinorRadius * [Math]::Sin($v)),
                ($Center[1] + $radial * [Math]::Cos($u)),
                ($Center[2] + $radial * [Math]::Sin($u))
            )
            $normal = [double[]]@([Math]::Sin($v), ([Math]::Cos($v) * [Math]::Cos($u)), ([Math]::Cos($v) * [Math]::Sin($u)))
            $ring += Add-Vertex $Mesh $position $normal
        }
        $rings += ,$ring
    }
    for ($major = 0; $major -lt $MajorSegments; $major++) {
        $nextMajor = ($major + 1) % $MajorSegments
        for ($minor = 0; $minor -lt $MinorSegments; $minor++) {
            $nextMinor = ($minor + 1) % $MinorSegments
            Add-QuadFacing $Mesh $rings[$major][$minor] $rings[$nextMajor][$minor] $rings[$nextMajor][$nextMinor] $rings[$major][$nextMinor]
        }
    }
}

$materials = @(
    [ordered]@{ name = "Airframe Pearl"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.94, 0.965, 1.0, 1.0); metallicFactor = 0.0; roughnessFactor = 0.26 } },
    [ordered]@{ name = "Competition Red"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.88, 0.018, 0.028, 1.0); metallicFactor = 0.0; roughnessFactor = 0.24 } },
    [ordered]@{ name = "Deep Navy Accent"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.012, 0.035, 0.11, 1.0); metallicFactor = 0.0; roughnessFactor = 0.30 } },
    [ordered]@{ name = "Tinted Canopy"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.018, 0.095, 0.17, 1.0); metallicFactor = 0.0; roughnessFactor = 0.11 } },
    [ordered]@{ name = "Painted Spinner"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.92, 0.022, 0.018, 1.0); metallicFactor = 0.0; roughnessFactor = 0.19 } },
    [ordered]@{ name = "Carbon Propeller"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.014, 0.017, 0.024, 1.0); metallicFactor = 0.12; roughnessFactor = 0.30 } },
    [ordered]@{ name = "Gear Metal"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.42, 0.46, 0.52, 1.0); metallicFactor = 0.78; roughnessFactor = 0.27 } },
    [ordered]@{ name = "Tire Rubber"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.012, 0.014, 0.018, 1.0); metallicFactor = 0.0; roughnessFactor = 0.88 } }
)

$meshes = [System.Collections.Generic.List[object]]::new()

$fuselage = New-Mesh "Fuselage" 0
Add-LoftZ $fuselage @(
    @(-0.46, 0.012, 0.166, 0.162), @(-0.34, 0.016, 0.179, 0.178),
    @(-0.16, 0.022, 0.183, 0.190), @(0.04, 0.032, 0.171, 0.183),
    @(0.22, 0.043, 0.150, 0.164), @(0.40, 0.056, 0.122, 0.139),
    @(0.56, 0.071, 0.092, 0.112), @(0.70, 0.086, 0.064, 0.086),
    @(0.81, 0.099, 0.039, 0.057), @(0.88, 0.108, 0.019, 0.030),
    @(0.91, 0.112, 0.008, 0.012)
) 48
$meshes.Add($fuselage)

$cowl = New-Mesh "Cowl" 1
Add-LoftZ $cowl @(@(-0.70, 0.006, 0.116, 0.118), @(-0.655, 0.007, 0.145, 0.145), @(-0.585, 0.009, 0.170, 0.164), @(-0.50, 0.011, 0.176, 0.168), @(-0.43, 0.013, 0.172, 0.165)) 48
$meshes.Add($cowl)

$spinner = New-Mesh "Spinner" 4
Add-LoftZ $spinner @(@(-0.89, 0.006, 0.006, 0.006), @(-0.855, 0.006, 0.032, 0.032), @(-0.81, 0.006, 0.066, 0.066), @(-0.755, 0.006, 0.096, 0.096), @(-0.705, 0.006, 0.114, 0.114), @(-0.678, 0.006, 0.118, 0.118)) 48
$meshes.Add($spinner)

$propeller = New-Mesh "Propeller" 5
Add-TwistedPropellerBlade $propeller 1.0
Add-TwistedPropellerBlade $propeller -1.0
$meshes.Add($propeller)

$canopy = New-Mesh "Canopy" 3
Add-LoftZ $canopy @(@(-0.35, 0.145, 0.018, 0.016), @(-0.31, 0.165, 0.070, 0.055), @(-0.23, 0.190, 0.115, 0.095), @(-0.10, 0.207, 0.137, 0.124), @(0.04, 0.210, 0.133, 0.128), @(0.17, 0.195, 0.108, 0.105), @(0.27, 0.165, 0.061, 0.058), @(0.32, 0.140, 0.016, 0.014)) 40
$meshes.Add($canopy)

$wing = New-Mesh "MainWingFixed" 0
Add-AirfoilPanel $wing @(@(-0.015, 0.020, -0.300, 0.260, 0.086), @(-0.16, 0.022, -0.296, 0.254, 0.083), @(-0.30, 0.027, -0.286, 0.242, 0.076))
Add-AirfoilPanel $wing @(@(-0.30, 0.027, -0.286, 0.105, 0.076), @(-0.50, 0.036, -0.270, 0.104, 0.066), @(-0.70, 0.048, -0.238, 0.102, 0.050), @(-0.84, 0.059, -0.202, 0.100, 0.030), @(-0.92, 0.066, -0.168, 0.096, 0.012))
Add-AirfoilPanel $wing @(@(0.015, 0.020, -0.300, 0.260, 0.086), @(0.16, 0.022, -0.296, 0.254, 0.083), @(0.30, 0.027, -0.286, 0.242, 0.076))
Add-AirfoilPanel $wing @(@(0.30, 0.027, -0.286, 0.105, 0.076), @(0.50, 0.036, -0.270, 0.104, 0.066), @(0.70, 0.048, -0.238, 0.102, 0.050), @(0.84, 0.059, -0.202, 0.100, 0.030), @(0.92, 0.066, -0.168, 0.096, 0.012))
$meshes.Add($wing)

$leftAileron = New-Mesh "LeftAileron" 1
Add-AirfoilPanel $leftAileron @(@(-0.30, 0.027, 0.112, 0.242, 0.036), @(-0.50, 0.036, 0.111, 0.238, 0.032), @(-0.70, 0.048, 0.109, 0.224, 0.025), @(-0.84, 0.059, 0.106, 0.190, 0.017), @(-0.90, 0.065, 0.103, 0.160, 0.009))
$meshes.Add($leftAileron)

$rightAileron = New-Mesh "RightAileron" 1
Add-AirfoilPanel $rightAileron @(@(0.30, 0.027, 0.112, 0.242, 0.036), @(0.50, 0.036, 0.111, 0.238, 0.032), @(0.70, 0.048, 0.109, 0.224, 0.025), @(0.84, 0.059, 0.106, 0.190, 0.017), @(0.90, 0.065, 0.103, 0.160, 0.009))
$meshes.Add($rightAileron)

$horizontalTail = New-Mesh "HorizontalStabilizer" 0
Add-AirfoilPanel $horizontalTail @(@(-0.012, 0.112, 0.535, 0.895, 0.040), @(-0.18, 0.112, 0.545, 0.885, 0.036))
Add-AirfoilPanel $horizontalTail @(@(-0.18, 0.112, 0.545, 0.730, 0.036), @(-0.48, 0.122, 0.600, 0.730, 0.014), @(-0.53, 0.125, 0.635, 0.730, 0.008))
Add-AirfoilPanel $horizontalTail @(@(0.012, 0.112, 0.535, 0.895, 0.040), @(0.18, 0.112, 0.545, 0.885, 0.036))
Add-AirfoilPanel $horizontalTail @(@(0.18, 0.112, 0.545, 0.730, 0.036), @(0.48, 0.122, 0.600, 0.730, 0.014), @(0.53, 0.125, 0.635, 0.730, 0.008))
$meshes.Add($horizontalTail)

$elevator = New-Mesh "Elevator" 2
Add-AirfoilPanel $elevator @(@(-0.50, 0.123, 0.735, 0.805, 0.010), @(-0.18, 0.112, 0.735, 0.885, 0.022), @(0.0, 0.112, 0.735, 0.900, 0.024), @(0.18, 0.112, 0.735, 0.885, 0.022), @(0.50, 0.123, 0.735, 0.805, 0.010))
$meshes.Add($elevator)

$verticalTail = New-Mesh "VerticalStabilizer" 0
Add-AirfoilPanel $verticalTail @(@(0.105, 0.0, 0.505, 0.895, 0.042), @(0.22, 0.0, 0.525, 0.865, 0.035)) -Plane vertical
Add-AirfoilPanel $verticalTail @(@(0.22, 0.0, 0.525, 0.730, 0.035), @(0.43, 0.0, 0.595, 0.730, 0.020), @(0.54, 0.0, 0.680, 0.730, 0.008)) -Plane vertical
$meshes.Add($verticalTail)

$rudder = New-Mesh "Rudder" 1
Add-AirfoilPanel $rudder @(@(0.12, 0.0, 0.735, 0.900, 0.026), @(0.30, 0.0, 0.735, 0.855, 0.021), @(0.51, 0.0, 0.735, 0.785, 0.009)) -Plane vertical
$meshes.Add($rudder)

$mainGear = New-Mesh "MainLandingGear" 6
Add-Cylinder $mainGear ([double[]]@(-0.115, -0.045, 0.015)) ([double[]]@(-0.275, -0.285, 0.045)) 0.012 12
Add-Cylinder $mainGear ([double[]]@(0.115, -0.045, 0.015)) ([double[]]@(0.275, -0.285, 0.045)) 0.012 12
Add-Cylinder $mainGear ([double[]]@(-0.315, -0.285, 0.045)) ([double[]]@(-0.235, -0.285, 0.045)) 0.018 16
Add-Cylinder $mainGear ([double[]]@(0.235, -0.285, 0.045)) ([double[]]@(0.315, -0.285, 0.045)) 0.018 16
$meshes.Add($mainGear)

$noseGear = New-Mesh "NoseLandingGear" 6
Add-Cylinder $noseGear ([double[]]@(0.0, -0.075, -0.455)) ([double[]]@(0.0, -0.265, -0.500)) 0.010 12
Add-Cylinder $noseGear ([double[]]@(-0.035, -0.265, -0.500)) ([double[]]@(0.035, -0.265, -0.500)) 0.015 14
$meshes.Add($noseGear)

$wheels = New-Mesh "Wheels" 7
Add-TorusX $wheels ([double[]]@(-0.275, -0.285, 0.045)) 0.052 0.017 32 12
Add-TorusX $wheels ([double[]]@(0.275, -0.285, 0.045)) 0.052 0.017 32 12
Add-TorusX $wheels ([double[]]@(0.0, -0.265, -0.500)) 0.038 0.014 28 10
$meshes.Add($wheels)

$livery = New-Mesh "WingAndFuselageLivery" 2
Add-AirfoilPanel $livery @(@(-0.895, 0.067, -0.188, 0.095, 0.009), @(-0.70, 0.055, -0.225, 0.095, 0.009))
Add-AirfoilPanel $livery @(@(0.70, 0.055, -0.225, 0.095, 0.009), @(0.895, 0.067, -0.188, 0.095, 0.009))
Add-LoftZ $livery @(@(-0.415, 0.020, 0.176, 0.170), @(-0.385, 0.020, 0.178, 0.171)) 28
$meshes.Add($livery)

# Additional G3C-B graphics are appended after the original 16 primitives so
# the authored G1E articulation indices 6/7/9/11 remain stable.
$topRedLivery = New-Mesh "TopRedLivery" 1
Add-AirfoilPanel $topRedLivery @(@(-0.32, 0.068, -0.275, -0.135, 0.006), @(-0.58, 0.071, -0.252, -0.125, 0.006), @(-0.84, 0.078, -0.198, -0.105, 0.005))
Add-AirfoilPanel $topRedLivery @(@(0.32, 0.068, -0.275, -0.135, 0.006), @(0.58, 0.071, -0.252, -0.125, 0.006), @(0.84, 0.078, -0.198, -0.105, 0.005))
$meshes.Add($topRedLivery)

$underside = New-Mesh "UndersideNavyLivery" 2
Add-AirfoilPanel $underside @(@(-0.30, 0.007, -0.260, 0.085, 0.006), @(-0.52, 0.018, -0.245, 0.083, 0.006), @(-0.72, 0.035, -0.215, 0.080, 0.005), @(-0.88, 0.052, -0.170, 0.075, 0.004))
Add-AirfoilPanel $underside @(@(0.30, 0.007, -0.260, 0.085, 0.006), @(0.52, 0.018, -0.245, 0.083, 0.006), @(0.72, 0.035, -0.215, 0.080, 0.005), @(0.88, 0.052, -0.170, 0.075, 0.004))
$meshes.Add($underside)

$canopyFrame = New-Mesh "CanopyFrame" 2
Add-LoftZ $canopyFrame @(@(-0.315, 0.164, 0.072, 0.058), @(-0.292, 0.174, 0.089, 0.072)) 40
Add-LoftZ $canopyFrame @(@(0.205, 0.182, 0.090, 0.087), @(0.228, 0.176, 0.079, 0.075)) 40
$meshes.Add($canopyFrame)

$wheelHubs = New-Mesh "WheelHubs" 6
Add-Cylinder $wheelHubs ([double[]]@(-0.299, -0.285, 0.045)) ([double[]]@(-0.251, -0.285, 0.045)) 0.025 24
Add-Cylinder $wheelHubs ([double[]]@(0.251, -0.285, 0.045)) ([double[]]@(0.299, -0.285, 0.045)) 0.025 24
Add-Cylinder $wheelHubs ([double[]]@(-0.018, -0.265, -0.500)) ([double[]]@(0.018, -0.265, -0.500)) 0.018 20
$meshes.Add($wheelHubs)

$propellerTips = New-Mesh "PropellerTips" 1
Add-ExtrudedPolygonXY $propellerTips @(@(-0.018, 0.330), @(-0.010, 0.372), @(0.010, 0.372), @(0.022, 0.330)) -0.706 -0.680
Add-ExtrudedPolygonXY $propellerTips @(@(0.018, -0.330), @(0.010, -0.372), @(-0.010, -0.372), @(-0.022, -0.330)) -0.706 -0.680
$meshes.Add($propellerTips)

foreach ($mesh in $meshes) {
    if ($mesh.Positions.Count -eq 0 -or $mesh.Positions.Count -ne $mesh.Normals.Count -or ($mesh.Indices.Count % 3) -ne 0) {
        throw "Invalid generated mesh $($mesh.Name)"
    }
}

$binaryStream = [System.IO.MemoryStream]::new()
$binaryWriter = [System.IO.BinaryWriter]::new($binaryStream)
$bufferViews = [System.Collections.Generic.List[object]]::new()
$accessors = [System.Collections.Generic.List[object]]::new()
$gltfMeshes = [System.Collections.Generic.List[object]]::new()
$nodes = [System.Collections.Generic.List[object]]::new()
$sceneNodes = [System.Collections.Generic.List[int]]::new()

function Align-Binary {
    while (($binaryStream.Length % 4) -ne 0) { $binaryWriter.Write([byte]0) }
}

for ($meshIndex = 0; $meshIndex -lt $meshes.Count; $meshIndex++) {
    $mesh = $meshes[$meshIndex]
    Align-Binary
    $positionOffset = [int]$binaryStream.Position
    foreach ($value in $mesh.Positions) { $binaryWriter.Write([float]$value) }
    $positionLength = [int]$binaryStream.Position - $positionOffset
    $positionView = $bufferViews.Count
    $bufferViews.Add([ordered]@{ buffer = 0; byteOffset = $positionOffset; byteLength = $positionLength; target = 34962 })
    $positionAccessor = $accessors.Count
    $xs = @(); $ys = @(); $zs = @()
    for ($index = 0; $index -lt $mesh.Positions.Count; $index += 3) {
        $xs += $mesh.Positions[$index]; $ys += $mesh.Positions[$index + 1]; $zs += $mesh.Positions[$index + 2]
    }
    $accessors.Add([ordered]@{
        bufferView = $positionView; componentType = 5126; count = [int]($mesh.Positions.Count / 3); type = "VEC3"
        min = @([float](($xs | Measure-Object -Minimum).Minimum), [float](($ys | Measure-Object -Minimum).Minimum), [float](($zs | Measure-Object -Minimum).Minimum))
        max = @([float](($xs | Measure-Object -Maximum).Maximum), [float](($ys | Measure-Object -Maximum).Maximum), [float](($zs | Measure-Object -Maximum).Maximum))
    })

    Align-Binary
    $normalOffset = [int]$binaryStream.Position
    foreach ($value in $mesh.Normals) { $binaryWriter.Write([float]$value) }
    $normalLength = [int]$binaryStream.Position - $normalOffset
    $normalView = $bufferViews.Count
    $bufferViews.Add([ordered]@{ buffer = 0; byteOffset = $normalOffset; byteLength = $normalLength; target = 34962 })
    $normalAccessor = $accessors.Count
    $accessors.Add([ordered]@{ bufferView = $normalView; componentType = 5126; count = [int]($mesh.Normals.Count / 3); type = "VEC3" })

    Align-Binary
    $indexOffset = [int]$binaryStream.Position
    foreach ($value in $mesh.Indices) { $binaryWriter.Write([uint32]$value) }
    $indexLength = [int]$binaryStream.Position - $indexOffset
    $indexView = $bufferViews.Count
    $bufferViews.Add([ordered]@{ buffer = 0; byteOffset = $indexOffset; byteLength = $indexLength; target = 34963 })
    $indexAccessor = $accessors.Count
    $accessors.Add([ordered]@{ bufferView = $indexView; componentType = 5125; count = $mesh.Indices.Count; type = "SCALAR" })

    $primitive = [ordered]@{
        attributes = [ordered]@{ POSITION = $positionAccessor; NORMAL = $normalAccessor }
        indices = $indexAccessor
        material = $mesh.Material
        mode = 4
    }
    $gltfMeshes.Add([ordered]@{ name = $mesh.Name; primitives = @($primitive) })
    $nodes.Add([ordered]@{ name = $mesh.Name; mesh = $meshIndex })
    $sceneNodes.Add($meshIndex)
}

Align-Binary
$binary = $binaryStream.ToArray()
$jsonObject = [ordered]@{
    asset = [ordered]@{
        version = "2.0"
        generator = "RC Simulation Engine G3C-B deterministic aircraft generator v2"
        extras = [ordered]@{
            foundation = "G3C-B production visual closure"
            coordinates = "+X right, +Y up, -Z forward/nose"
            provenance = "Original procedural geometry; repository MIT license"
        }
    }
    scene = 0
    scenes = @([ordered]@{ name = "Acro Electric 01"; nodes = @($sceneNodes) })
    nodes = @($nodes)
    meshes = @($gltfMeshes)
    materials = $materials
    buffers = @([ordered]@{ byteLength = $binary.Length })
    bufferViews = @($bufferViews)
    accessors = @($accessors)
}
$json = $jsonObject | ConvertTo-Json -Depth 20 -Compress
$jsonBytes = [System.Text.Encoding]::UTF8.GetBytes($json)
$jsonPadding = (4 - ($jsonBytes.Length % 4)) % 4
$totalLength = 12 + 8 + $jsonBytes.Length + $jsonPadding + 8 + $binary.Length

$outputDirectory = Split-Path -Parent $OutputPath
if ($outputDirectory) { [System.IO.Directory]::CreateDirectory($outputDirectory) | Out-Null }
$output = [System.IO.File]::Open($OutputPath, [System.IO.FileMode]::Create)
$writer = [System.IO.BinaryWriter]::new($output)
$writer.Write([uint32]0x46546C67); $writer.Write([uint32]2); $writer.Write([uint32]$totalLength)
$writer.Write([uint32]($jsonBytes.Length + $jsonPadding)); $writer.Write([uint32]0x4E4F534A); $writer.Write($jsonBytes)
for ($index = 0; $index -lt $jsonPadding; $index++) { $writer.Write([byte]0x20) }
$writer.Write([uint32]$binary.Length); $writer.Write([uint32]0x004E4942); $writer.Write($binary)
$writer.Dispose(); $binaryWriter.Dispose(); $binaryStream.Dispose()

$vertexCount = ($meshes | ForEach-Object { $_.Positions.Count / 3 } | Measure-Object -Sum).Sum
$triangleCount = ($meshes | ForEach-Object { $_.Indices.Count / 3 } | Measure-Object -Sum).Sum
Write-Output "Generated $OutputPath ($totalLength bytes, $vertexCount vertices, $triangleCount triangles, $($meshes.Count) primitives, $($materials.Count) materials)"
