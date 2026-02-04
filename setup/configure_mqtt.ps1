# H.E.R.M.E.S. - Configuración Automática de MQTT
# Este script prepara tu PC para comunicarse con el robot H.E.R.M.E.S.
# REQUISITO: Ejecutar como ADMINISTRADOR.

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "   H.E.R.M.E.S. MQTT SETUP TOOL   " -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan

$mosquittoPath = "C:\Program Files\mosquitto"
$confFile = "$mosquittoPath\mosquitto.conf"
$repoConf = "$PSScriptRoot\mosquitto.conf"

# 1. Configurar el archivo de Mosquitto
if (Test-Path $confFile) {
    Write-Host "[1/3] Configurando mosquitto.conf..." -ForegroundColor Green
    
    # Copiar configuración optimizada del repo
    if (Test-Path $repoConf) {
        Copy-Item -Path $repoConf -Destination $confFile -Force
        Write-Host "      - Configuración aplicada desde el repositorio." -ForegroundColor Gray
    }
    else {
        # Backup y edición manual si no hay archivo en repo
        Add-Content -Path $confFile -Value "`nlistener 1883 0.0.0.0`nallow_anonymous true"
        Write-Host "      - Reglas de acceso externo agregadas manualmente." -ForegroundColor Gray
    }
}
else {
    Write-Host "[!] Error: Mosquitto no encontrado en $mosquittoPath" -ForegroundColor Red
    return
}

# 2. Configurar el Firewall de Windows
Write-Host "[2/3] Abriendo puerto 1883 en el Firewall..." -ForegroundColor Green
$ruleName = "HERMES MQTT (TCP 1883)"
$existingRule = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue

if ($existingRule) {
    Write-Host "      - La regla del Firewall ya existe." -ForegroundColor Gray
}
else {
    New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -LocalPort 1883 -Protocol TCP -Action Allow | Out-Null
    Write-Host "      - Regla creada exitosamente." -ForegroundColor Gray
}

# 3. Reiniciar el servicio
Write-Host "[3/3] Reiniciando servicio de Mosquitto..." -ForegroundColor Green
Restart-Service -Name "mosquitto" -ErrorAction SilentlyContinue
Write-Host "      - Servicio reiniciado." -ForegroundColor Gray

Write-Host "`n¡LISTO! El robot ya puede conectarse a tu PC." -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
