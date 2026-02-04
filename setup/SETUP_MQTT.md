# Guía de Configuración MQTT Paso a Paso 📡

Esta guía explica cómo configurar tu computadora para que el robot H.E.R.M.E.S. pueda comunicarse con ella sin importar en qué red WiFi estés.

## 🚀 Método Rápido (Recomendado)

Hemos creado un script que hace todo el trabajo pesado por ti (Configurar Mosquitto y abrir el Firewall).

1.  Abre la carpeta `setup/` en tu computadora.
2.  Haz clic derecho sobre `configure_mqtt.ps1`.
3.  Selecciona **"Ejecutar con PowerShell"**.
4.  Si Windows te pide permisos de Administrador, dile que **SÍ**.

---

## 🛠️ Método Manual (Si el script falla)

Si prefieres hacerlo tú mismo o el script no funciona, sigue estos pasos:

### 1. Configurar Mosquitto
Debes permitir que Mosquitto acepte conexiones desde otros dispositivos (como el robot).

1.  Abre el **Bloc de Notas** como **Administrador**.
2.  Abre el archivo: `C:\Program Files\mosquitto\mosquitto.conf`.
3.  Baja hasta el final y agrega estas líneas:
    ```text
    listener 1883 0.0.0.0
    allow_anonymous true
    ```
4.  Guarda el archivo.

### 2. Abrir el Firewall de Windows
Windows bloquea por defecto el puerto 1883. Debes crear una excepción.

1.  Abre **PowerShell** como **Administrador**.
2.  Ejecuta este comando:
    ```powershell
    New-NetFirewallRule -DisplayName "HERMES MQTT" -Direction Inbound -LocalPort 1883 -Protocol TCP -Action Allow
    ```

### 3. Reiniciar el Servicio
Para que los cambios surtan efecto, reinicia el broker.

1.  Abre el **Administrador de Tareas** (`Ctrl+Shift+Esc`).
2.  Ve a la pestaña **Servicios**.
3.  Busca `mosquitto`, haz clic derecho y selecciona **Reiniciar**.

---

## ✅ ¿Cómo saber si funciona?

Mira la consola de Thonny mientras el robot enciende. Deberías ver:
`🟢 SUCCESS | Connected to [Tu WiFi]!`
`🟢 SUCCESS | MQTT Connected!`

Si ves eso, ¡tu configuración es perfecta! 🥳
