param(
    [Parameter(Mandatory = $true)]
    [string]$Svo,
    [string]$Python = "python"
)
$ErrorActionPreference = "Stop"
if ($Svo.StartsWith("\\")) {
    throw "Use the SDK PC's local SVO2 path; do not read the recording over the share."
}
if (-not (Test-Path -LiteralPath $Svo -PathType Leaf)) {
    throw "SVO2 not found: $Svo"
}
$Bundle = Join-Path (Split-Path -Parent $PSScriptRoot) "20261021_group002_recording_003_skeleton_replay"
$Exporter = Join-Path $PSScriptRoot "analyze_svo_body_tracking_recording003_20261021.py"
$Builder = Join-Path $Bundle "build_walk_skeleton_replay.py"
foreach ($Required in @($Exporter, $Builder, (Join-Path $Bundle "bundle.json"))) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) {
        throw "Missing prepared file: $Required"
    }
}
$Stamp = Get-Date -Format "yyyyMMdd_HHmmss_fff"
$BodyOut = Join-Path $Bundle "exports\body_$Stamp"
$HtmlOut = Join-Path $Bundle "exports\walk_skeleton_$Stamp.html"
Write-Host "Reading local SVO2: $Svo"
Write-Host "New SDK output: $BodyOut"
& $Python $Exporter $Svo --output $BodyOut --max-depth-m 30 --confidence 35 --preview-period-s 0.5
if ($LASTEXITCODE -ne 0) {
    throw "ZED body export failed. No completed skeleton playback was generated. See SDK error above."
}
& $Python $Builder --bundle-dir $Bundle --body-dir $BodyOut --output $HtmlOut
if ($LASTEXITCODE -ne 0) {
    throw "Playback generation failed. SDK output has been preserved: $BodyOut"
}
Write-Host "Completed playback: $HtmlOut"
Start-Process -FilePath $HtmlOut
