# GUÍA DE INSTALACIÓN Y PRUEBAS - HERMES GCS v2.2 (Final)

Esta guía es para configurar el robot (ESP32) con el nuevo código mejorado.

## PASO 1: Preparar el ESP32

Necesitarás un cable USB y un PC con **Thonny IDE** instalado (o cualquier herramienta para subir archivos a MicroPython).

1.  **Conecta el ESP32** al computador por USB.
2.  Abre **Thonny IDE**.
3.  En la esquina inferior derecha, selecciona tu dispositivo (ej. "MicroPython (ESP32)").

## PASO 2: Configurar el WiFi y MQTT

Antes de subir los archivos, debes poner los datos de TU red.

1.  Abre el archivo `firmware/config.py` que está en esta carpeta.
2.  Edita estas líneas con tus datos reales:
    ```python
    WIFI_SSID = "NOMBRE_DE_TU_WIFI"
    WIFI_PASSWORD = "TU_CONTRASEÑA"
    
    # IMPORTANTE: Pon la dirección IP de la computadora que corre el GCS (el programa de control)
    MQTT_BROKER = "192.168.1.XX" 
    
    # CONFIGURACIÓN SEGURA (Puesto por defecto en v2.2)
    MQTT_USER = "hermes_robot"
    MQTT_PASSWORD = "hermes_password"
    ```
    *Nota: Si tienes una ESP32-CAM aparte, asegúrate de configurar su IP en el archivo `config.json` o `src/config.py` de la carpeta GCS.*
3.  Guarda el archivo.

## PASO 3: Subir los Archivos al Robot

Debes copiar los 4 archivos de la carpeta `firmware/` a la raíz del ESP32.

Los archivos son:
1.  `config.py` (¡Con tus cambios del Paso 2!)
2.  `drivers.py`
3.  `main.py`
4.  `pid.py`    - Control de estabilidad PID.
5.  `MPU6050.py` - Driver del sensor inercial.

**En Thonny:**
*   A la izquierda verás "Este ordenador" y la carpeta `firmware`.
*   A la izquierda abajo (o donde diga "Dispositivo MicroPython"), haz clic derecho y selecciona "Upload" o simplemente guarda cada archivo en el dispositivo con el mismo nombre.
*   **Asegúrate de que queden guardados en el ESP32.**

## PASO 4: Prueba Inicial (Sin GCS)

1.  En Thonny, presiona el botón **Reset** del ESP32 (o desconecta y conecta el USB).
2.  Mira la consola de Thonny (abajo). Deberías ver algo así:
    ```
    [   1.42] 🟢 SUCCESS | Config loaded OK (v2.2)
    [   2.10] 🔵 INFO    | Connecting to WiFi...
    [   5.32] 🟢 SUCCESS | Connected: 192.168.1.45  <-- ¡Asegúrate que diga Connected!
    [   6.15] 🔵 INFO    | Connecting to MQTT broker...
    [   7.20] 🟢 SUCCESS | MQTT Connected!
    ```
    *   Si ves los **iconos de colores**, el nuevo sistema de logs estructurados está funcionando.

## PASO 5: Usar el GCS (Programa de Control)

1.  En tu computadora, ejecuta el programa principal:
    ```bash
    python main.py
    ```
2.  Abre el navegador en `http://127.0.0.1:8050`.
3.  Deberías ver el indicador de estado "ONLINE" en verde (arriba a la derecha).
4.  Prueba mover el robot con los botones o el teclado.
5.  Prueba el slider de intensidad LED.

¡Listo! El robot ahora responde más rápido y es más seguro.
