param(
    [string]$InputFile = (Join-Path $PSScriptRoot "..\ARC574_F26_RoboticBrickAssembly_IRB4600_Rhino8_Template.gh"),
    [string]$OutputDirectory = (Join-Path $PSScriptRoot "..\reference\grasshopper_extracted")
)

$ErrorActionPreference = "Stop"

$ghIo = "C:\Program Files\Rhino 8\Plug-ins\Grasshopper\GH_IO.dll"
if (-not (Test-Path -LiteralPath $ghIo)) {
    throw "GH_IO.dll was not found at $ghIo"
}

Add-Type -Path $ghIo

$archive = New-Object GH_IO.Serialization.GH_Archive
if (-not $archive.ReadFromFile((Resolve-Path -LiteralPath $InputFile))) {
    throw "Grasshopper archive could not be read: $InputFile"
}

[xml]$document = $archive.Serialize_Xml()
$scriptObjects = @(
    $document.SelectNodes("//chunk[@name='Object']") |
        Where-Object { $_.SelectSingleNode(".//chunk[@name='Script']") }
)

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null

$nameCounts = @{}
$manifest = @()
for ($index = 0; $index -lt $scriptObjects.Count; $index++) {
    $ordinal = $index + 1
    $object = $scriptObjects[$index]
    $script = $object.SelectSingleNode(".//chunk[@name='Script']")
    $title = $script.SelectSingleNode(".//item[@name='Title']").InnerText.Trim()
    $encoded = $script.SelectSingleNode(".//item[@name='Text']").InnerText.Trim()
    $source = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($encoded))

    $slug = ($title.ToLowerInvariant() -replace '[^a-z0-9]+', '_').Trim('_')
    if (-not $slug) { $slug = "python" }
    if (-not $nameCounts.ContainsKey($slug)) { $nameCounts[$slug] = 0 }
    $nameCounts[$slug]++
    $fileName = "{0:d2}_{1}_{2:d2}.py" -f $ordinal, $slug, $nameCounts[$slug]

    $excluded = $ordinal -ge 21
    $sha256 = [Security.Cryptography.SHA256]::Create()
    try {
        $hashBytes = $sha256.ComputeHash([Text.Encoding]::UTF8.GetBytes($source))
    }
    finally {
        $sha256.Dispose()
    }
    $hash = ([BitConverter]::ToString($hashBytes) -replace '-', '').ToLowerInvariant()

    if (-not $excluded) {
        [IO.File]::WriteAllText(
            (Join-Path $OutputDirectory $fileName),
            $source,
            (New-Object Text.UTF8Encoding($false))
        )
    }

    $manifest += [pscustomobject]@{
        index = $ordinal
        title = $title
        extracted_file = $(if ($excluded) { $null } else { $fileName })
        excluded = $excluded
        exclusion_reason = $(if ($excluded) { "brick graph logic deferred by user" } else { $null })
        sha256 = $hash
        characters = $source.Length
    }
}

$manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $OutputDirectory "manifest.json") -Encoding utf8

Write-Host "Read $($scriptObjects.Count) embedded scripts."
Write-Host "Extracted scripts 1-20 to $OutputDirectory."
Write-Host "Scripts 21-24 were inventoried but excluded."
