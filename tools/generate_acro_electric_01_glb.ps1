param(
    [string]$OutputPath = "models/acro_electric_01/aircraft.glb"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# Deterministic, dependency-free authoring source for the G3C-A visual asset.
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
    foreach ($value in $Position) { $Mesh.Positions.Add([float]$value) }
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
        @(0.00, 0.00), @(0.08, 0.34), @(0.28, 0.50), @(0.58, 0.38),
        @(0.84, 0.18), @(1.00, 0.00), @(0.82, -0.12), @(0.48, -0.20), @(0.16, -0.14)
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
    [ordered]@{ name = "Airframe Pearl"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.91, 0.94, 0.98, 1.0); metallicFactor = 0.05; roughnessFactor = 0.30 } },
    [ordered]@{ name = "Competition Red"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.82, 0.025, 0.035, 1.0); metallicFactor = 0.08; roughnessFactor = 0.28 } },
    [ordered]@{ name = "Deep Navy Accent"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.018, 0.055, 0.13, 1.0); metallicFactor = 0.06; roughnessFactor = 0.32 } },
    [ordered]@{ name = "Tinted Canopy"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.025, 0.13, 0.22, 1.0); metallicFactor = 0.18; roughnessFactor = 0.16 } },
    [ordered]@{ name = "Anodized Spinner"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.88, 0.035, 0.025, 1.0); metallicFactor = 0.62; roughnessFactor = 0.20 } },
    [ordered]@{ name = "Carbon Propeller"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.018, 0.021, 0.026, 1.0); metallicFactor = 0.24; roughnessFactor = 0.25 } },
    [ordered]@{ name = "Gear Metal"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.42, 0.46, 0.52, 1.0); metallicFactor = 0.78; roughnessFactor = 0.27 } },
    [ordered]@{ name = "Tire Rubber"; pbrMetallicRoughness = [ordered]@{ baseColorFactor = @(0.012, 0.014, 0.018, 1.0); metallicFactor = 0.0; roughnessFactor = 0.88 } }
)

$meshes = [System.Collections.Generic.List[object]]::new()

$fuselage = New-Mesh "Fuselage" 0
Add-LoftZ $fuselage @(
    @(-0.46, 0.015, 0.168, 0.165), @(-0.20, 0.018, 0.174, 0.178),
    @(0.12, 0.030, 0.155, 0.160), @(0.40, 0.052, 0.112, 0.125),
    @(0.66, 0.078, 0.066, 0.083), @(0.82, 0.098, 0.025, 0.036)
) 28
$meshes.Add($fuselage)

$cowl = New-Mesh "Cowl" 1
Add-LoftZ $cowl @(@(-0.68, 0.008, 0.128, 0.130), @(-0.60, 0.010, 0.165, 0.160), @(-0.43, 0.015, 0.174, 0.168)) 28
$meshes.Add($cowl)

$spinner = New-Mesh "Spinner" 4
Add-LoftZ $spinner @(@(-0.875, 0.006, 0.010, 0.010), @(-0.825, 0.006, 0.052, 0.052), @(-0.735, 0.006, 0.105, 0.105), @(-0.685, 0.006, 0.118, 0.118)) 28
$meshes.Add($spinner)

$propeller = New-Mesh "Propeller" 5
$blade = @(@(-0.030, 0.075), @(-0.048, 0.155), @(-0.035, 0.350), @(0.006, 0.365), @(0.030, 0.155), @(0.026, 0.075))
Add-ExtrudedPolygonXY $propeller $blade -0.704 -0.680
$oppositeBlade = foreach ($point in $blade) { ,([object[]]@(-[double]$point[0], -[double]$point[1])) }
Add-ExtrudedPolygonXY $propeller $oppositeBlade -0.704 -0.680
$meshes.Add($propeller)

$canopy = New-Mesh "Canopy" 3
Add-LoftZ $canopy @(@(-0.34, 0.145, 0.030, 0.030), @(-0.26, 0.165, 0.105, 0.085), @(-0.04, 0.185, 0.132, 0.125), @(0.18, 0.175, 0.100, 0.100), @(0.29, 0.135, 0.025, 0.025)) 24
$meshes.Add($canopy)

$wing = New-Mesh "MainWingFixed" 0
Add-AirfoilPanel $wing @(@(-0.015, 0.020, -0.285, 0.255, 0.080), @(-0.30, 0.026, -0.275, 0.245, 0.070))
Add-AirfoilPanel $wing @(@(-0.30, 0.026, -0.275, 0.105, 0.070), @(-0.68, 0.043, -0.245, 0.105, 0.050), @(-0.90, 0.058, -0.195, 0.105, 0.018))
Add-AirfoilPanel $wing @(@(0.015, 0.020, -0.285, 0.255, 0.080), @(0.30, 0.026, -0.275, 0.245, 0.070))
Add-AirfoilPanel $wing @(@(0.30, 0.026, -0.275, 0.105, 0.070), @(0.68, 0.043, -0.245, 0.105, 0.050), @(0.90, 0.058, -0.195, 0.105, 0.018))
$meshes.Add($wing)

$leftAileron = New-Mesh "LeftAileron" 1
Add-AirfoilPanel $leftAileron @(@(-0.30, 0.026, 0.110, 0.245, 0.036), @(-0.68, 0.043, 0.110, 0.235, 0.028), @(-0.875, 0.057, 0.110, 0.195, 0.014))
$meshes.Add($leftAileron)

$rightAileron = New-Mesh "RightAileron" 1
Add-AirfoilPanel $rightAileron @(@(0.30, 0.026, 0.110, 0.245, 0.036), @(0.68, 0.043, 0.110, 0.235, 0.028), @(0.875, 0.057, 0.110, 0.195, 0.014))
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
Add-TorusX $wheels ([double[]]@(-0.275, -0.285, 0.045)) 0.060 0.021 20 9
Add-TorusX $wheels ([double[]]@(0.275, -0.285, 0.045)) 0.060 0.021 20 9
Add-TorusX $wheels ([double[]]@(0.0, -0.265, -0.500)) 0.044 0.016 18 8
$meshes.Add($wheels)

$livery = New-Mesh "WingAndFuselageLivery" 2
Add-AirfoilPanel $livery @(@(-0.895, 0.067, -0.188, 0.095, 0.009), @(-0.70, 0.055, -0.225, 0.095, 0.009))
Add-AirfoilPanel $livery @(@(0.70, 0.055, -0.225, 0.095, 0.009), @(0.895, 0.067, -0.188, 0.095, 0.009))
Add-LoftZ $livery @(@(-0.415, 0.020, 0.176, 0.170), @(-0.385, 0.020, 0.178, 0.171)) 28
$meshes.Add($livery)

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
        generator = "RC Simulation Engine G3C-A deterministic aircraft generator v1"
        extras = [ordered]@{
            foundation = "G3C-A"
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
