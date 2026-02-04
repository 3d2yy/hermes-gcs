# H.E.R.M.E.S. MQTT Fix Script
# Run this as Administrator to allow the robot to connect to your PC.

$mosquittoPath = "C:\Program Files\mosquitto"
$confFile = "$mosquittoPath\mosquitto.conf"

if (Test-Path $confFile) {
    Write-Host "Configuring Mosquitto at $confFile..." -ForegroundColor Cyan
    
    # Check if listener 1883 and allow_anonymous true are already there
    $content = Get-Content $confFile
    $hasListener = $content | Select-String "listener 1883"
    $hasAnon = $content | Select-String "allow_anonymous true"

    if (!$hasListener -or !$hasAnon) {
        Write-Host "Adding external access rules..." -ForegroundColor Yellow
        Add-Content -Path $confFile -Value "`n# H.E.R.M.E.S. Config`nlistener 1883 0.0.0.0`nallow_anonymous true"
    } else {
        Write-Host "Configuration rules already present." -ForegroundColor Green
    }

    Write-Host "Restarting Mosquitto service..." -ForegroundColor Cyan
    Restart-Service -Name "mosquitto" -ErrorAction SilentlyContinue
    
    Write-Host "Done! The robot should be able to connect now." -ForegroundColor Green
} else {
    Write-Host "Mosquitto config not found at $confFile" -ForegroundColor Red
}
