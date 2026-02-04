# 🤖 Guía de Configuración ESP32 (MicroPython)

Esta guía explica cómo configurar el **cerebro del robot** (la placa ESP32 grande, no la cámara).

## 1. Requisitos
- Placa ESP32 (WROOM-32 o similar).
- Cable Micro-USB de datos.
- **Thonny IDE** instalado ([thonny.org](https://thonny.org/)).
- Python instalado.

## 2. Instalar el Firmware (MicroPython)
Si tu placa es nueva, necesitas intalarle MicroPython.
1. Descarga el firmware `.bin` desde [micropython.org/download/esp32/](https://micropython.org/download/esp32/).
2. Abre Thonny.
3. Ve a `Run` -> `Configure Interpreter`.
4. Selecciona "MicroPython (ESP32)" y el puerto COM de tu placa.
5. Haz clic en "Install or update MicroPython" (esquina inferior derecha).
6. Selecciona el archivo `.bin` descargado y espera a que termine.

## 3. Subir el Código del Proyecto
1. En Thonny, asegúrate de ver los archivos de tu PC a la izquierda y los de la ESP32 a la izquierda-abajo (Files).
2. Navega en tu PC a la carpeta `firmware/` de este proyecto.
3. Necesitas configurar tus claves WIFI antes de subir nada.
   - Copia el contenido de `firmware/config.py` a un nuevo archivo llamado `secrets.py`.
   - Edita `secrets.py` con tu **SSID** y **Password** de WiFi, y la IP de tu Broker MQTT.
   - **IMPORTANTE**: No subas `secrets.py` a GitHub si es un repo público.
4. Selecciona TODOS los archivos en la carpeta `firmware/` (excepto `secrets.py` si prefieres crearlo manualmente en el robot, pero es más fácil subirlo).
   - Incluye: `main.py`, `config.py`, `drivers.py`, `secrets.py` (con tus claves reales), y las carpetas de librerías si las hay (`umqtt`, `mpu6050`, etc).
5. Haz clic derecho -> **Upload to /**.

## 4. Estructura de Archivos en el Robot
Al final, tu ESP32 debería tener estos archivos:
- `boot.py` (Creado por defecto)
- `main.py` (Nuestro código principal)
- `config.py` (Configuración general)
- `secrets.py` (Tus claves privadas - NO COMPARTIR)
- `drivers.py` (Controladores de hardware)
- `pid.py`, `mpu6050.py`, `ssd1306.py` (Librerías necesarias)

## 5. Probar
Reinicia la placa (botón EN/RST). En la consola de Thonny deberías ver:
```
[BOOT] Config loaded OK
[BOOT] Connecting to WiFi...
[SUCCESS] Connected! IP: 192.168.1.XXX
```
¡Listo! Tu robot está vivo.
