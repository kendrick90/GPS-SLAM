# Run this script as Administrator in Windows PowerShell
# Right-click PowerShell -> Run as Administrator
# Then: .\attach_kinect_wsl.ps1

Write-Host "Attaching Azure Kinect to WSL..." -ForegroundColor Cyan

# List devices first
Write-Host "`nCurrent USB devices:" -ForegroundColor Yellow
usbipd list

# Find and attach Azure Kinect devices (VID 045e = Microsoft)
$devices = usbipd list | Select-String "045e:097[cd]"

foreach ($line in $devices) {
    if ($line -match "^(\d+-\d+)") {
        $busid = $matches[1]
        Write-Host "`nAttaching device $busid..." -ForegroundColor Green
        usbipd attach --wsl --busid $busid
    }
}

Write-Host "`nDone! Check WSL with: lsusb | grep -i kinect" -ForegroundColor Cyan
