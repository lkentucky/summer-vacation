param([switch]$InspectOnly)
$ErrorActionPreference = 'Stop'
$apiDir = 'D:/sw2026/SOLIDWORKS/api/redist'
Add-Type -Path "$apiDir/SolidWorks.Interop.sldworks.dll"
Add-Type -Path "$apiDir/SolidWorks.Interop.swconst.dll"
Add-Type -Path "$PSScriptRoot/BatteryBoxBuilder.cs" -ReferencedAssemblies "$apiDir/SolidWorks.Interop.sldworks.dll","$apiDir/SolidWorks.Interop.swconst.dll"
if ($InspectOnly) {
    [BatteryBoxBuilder]::Inspect()
} else {
    [BatteryBoxBuilder]::Build($PSScriptRoot)
}
