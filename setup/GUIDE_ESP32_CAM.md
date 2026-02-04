# 📸 Guía de Configuración ESP32-CAM (Arduino IDE)

Esta guía explica cómo programar el módulo de cámara.

## 1. Requisitos
- Módulo ESP32-CAM (AI Thinker).
- Módulo FTDI (USB a Serial) para programarlo (la ESP32-CAM no tiene USB).
- **Arduino IDE** instalado.

## 2. Preparar Arduino IDE
1. Abre Arduino IDE.
2. Ve a `Archivo` -> `Preferencias`.
3. En "Gestor de URLs Adicionales de Tarjetas", pega esto:
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```
4. Ve a `Herramientas` -> `Placa` -> `Gestor de Tarjetas`.
5. Busca "esp32" e instala la versión más reciente de **Espressif Systems**.

## 3. Configuración de la Placa
Selecciona en `Herramientas`:
- **Placa**: "AI Thinker ESP32-CAM"
- **CPU Frequency**: "240MHz (WiFi/BT)"
- **Flash Frequency**: "80MHz"
- **Flash Mode**: "QIO"
- **Partition Scheme**: "Huge APP (3MB No OTA/1MB SPIFFS)" <-- IMPORTANTE
- **Puerto**: El puerto COM de tu FTDI.

## 4. Subir el Código
1. Abre el archivo `camera_firmware/ESP32_CAM/ESP32_CAM.ino` en Arduino IDE.
2. Busca la sección de credenciales WiFi (al principio del archivo):
   ```cpp
   } networks[] = {
     {"YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD"}, // <--- EDITA ESTO
   };
   ```
3. Pon tu nombre de red y contraseña reales.
4. **IMPORTANTE**: Para subir el código, debes conectar el pin **IO0** a **GND** en la ESP32-CAM.
5. Presiona el botón RESET en la ESP32-CAM.
6. Dale al botón "Subir" (flecha) en Arduino IDE.
7. Cuando termine ("Subido"), **desconecta el cable entre IO0 y GND**.
8. Presiona RESET otra vez.
9. Abre el **Monitor Serie** (115200 baudios). Deberías ver:
   ```
   [WIFI] ¡Conectado!
   Camera Ready! Use 'http://192.168.1.XXX' to connect
   ```

¡Tu cámara está lista!
