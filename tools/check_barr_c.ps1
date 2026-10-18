$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$sourceFiles = Get-ChildItem -Path $projectRoot -Recurse -File |
    Where-Object {
        ($_.Extension -in ".c", ".h") -and
        ($_.FullName -notmatch "[\\/]build[^\\/]*[\\/]")
    }
$issueCount = 0

foreach ($sourceFile in $sourceFiles)
{
    $content = Get-Content -Raw -LiteralPath $sourceFile.FullName
    $relativePath = $sourceFile.FullName.Substring($projectRoot.Length + 1)

    if ($content.Contains("`t"))
    {
        Write-Error "$relativePath contains a tab character." -ErrorAction Continue
        $issueCount++
    }

    if ($content.Contains("`r`n"))
    {
        Write-Error "$relativePath contains CRLF line endings." -ErrorAction Continue
        $issueCount++
    }

    $lineNumber = 0

    foreach ($line in ($content -split "`n"))
    {
        $lineNumber++

        if (120 -lt $line.Length)
        {
            Write-Error "$relativePath`:$lineNumber exceeds 120 columns." `
                -ErrorAction Continue
            $issueCount++
        }
    }

    if ($sourceFile.BaseName -cne $sourceFile.BaseName.ToLowerInvariant())
    {
        Write-Error "$relativePath is not a lowercase module name." `
            -ErrorAction Continue
        $issueCount++
    }
}

$moduleSources = $sourceFiles | Where-Object { ".c" -eq $_.Extension }

foreach ($moduleSource in $moduleSources)
{
    $matchingHeader = Join-Path $moduleSource.DirectoryName `
        ($moduleSource.BaseName + ".h")

    if (-not (Test-Path -LiteralPath $matchingHeader))
    {
        $relativePath = $moduleSource.FullName.Substring(
            $projectRoot.Length + 1)
        Write-Error "$relativePath has no matching header." -ErrorAction Continue
        $issueCount++
    }
}

if (0 -ne $issueCount)
{
    throw "BARR-C check found $issueCount issue(s)."
}

Write-Output "BARR-C mechanical checks passed for $($sourceFiles.Count) files."
