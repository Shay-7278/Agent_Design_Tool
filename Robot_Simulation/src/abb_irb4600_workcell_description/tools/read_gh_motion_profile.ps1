param([string]$InputFile = (Join-Path $PSScriptRoot '..\ARC574_F26_RoboticBrickAssembly_IRB4600_Rhino8_Template.gh'))
$ErrorActionPreference = 'Stop'
Add-Type -Path 'C:\Program Files\Rhino 8\Plug-ins\Grasshopper\GH_IO.dll'
$archive = New-Object GH_IO.Serialization.GH_Archive
if (-not $archive.ReadFromFile((Resolve-Path -LiteralPath $InputFile))) { throw 'Cannot read GH archive' }
[xml]$document = $archive.Serialize_Xml()
function Container($id) {
    foreach ($object in $document.SelectNodes("//chunk[@name='Object']")) {
        $c = $object.SelectSingleNode("./chunks/chunk[@name='Container']")
        $g = $c.SelectSingleNode("./items/item[@name='InstanceGuid']")
        if ($g -and $g.InnerText -eq $id) { return $c }
    }
    throw "Missing GH object $id"
}
function Speed($id) {
    $selected = (Container $id).SelectNodes(".//chunk[@name='ListItem']") | Where-Object {
        $_.SelectSingleNode("./items/item[@name='Selected']").InnerText -eq 'true'
    }
    $expression = $selected.SelectSingleNode("./items/item[@name='Expression']").InnerText.Trim('"')
    if ($expression -notmatch '^v\d+$') { throw "Unsupported GH speed $expression" }
    return [double]$expression.Substring(1) / 1000.0
}
$coordinate = (Container '02db3e02-1a42-460f-ba0a-b334e40103a9').SelectSingleNode(".//item[@name='Coordinate']")
$ghHomePosition = @(([double]$coordinate.X / 1000), ([double]$coordinate.Y / 1000), ([double]$coordinate.Z / 1000))
$x = (Container '52eca75d-af91-4740-bfb7-e7c833393a7b').SelectSingleNode(".//item[@name='UserText']").InnerText.Trim()
$y = (Container 'e7d5d43b-950f-4607-9f25-a23e294b3ddf').SelectSingleNode(".//item[@name='UserText']").InnerText.Trim()
if ($x -ne '0,1,0' -or $y -ne '1,0,0') { throw 'GH home axes changed: re-extract its quaternion' }
$retract = $document.SelectSingleNode("//chunk[@name='Container'][items/item[@name='NickName']='Z Retract (mm)']/items/item[@name='UserText']")
$rail = (Container 'ca2121c7-1aba-4e8b-a6be-79d469e7759f').SelectSingleNode(".//item[@name='Value']")
[ordered]@{
    source = [IO.Path]::GetFileName($InputFile)
    source_sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $InputFile).Hash.ToLower()
    home_tcp = @{position = $ghHomePosition; orientation_xyzw = @(0.7071067811865476, 0.7071067811865475, 0.0, 0.0)}
    retract_m = [double]$retract.InnerText / 1000.0
    track_offset_m = [double]$rail.InnerText / 1000.0
    move_j_speed_m_s = Speed '19203653-d2fa-4594-bea8-d6b2606bd0ad'
    move_l_speed_m_s = Speed 'f8fe6087-ec65-45d5-aab0-7c6a7a295d7d'
    orientation_speed_rad_s = 500.0 * [Math]::PI / 180.0
    rail_speed_m_s = 5.0
    wait_s = 1.0
    cfx = 0
} | ConvertTo-Json -Depth 4
