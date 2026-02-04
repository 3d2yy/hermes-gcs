# 🦟 Guía de Configuración MQTT (Mosquitto)

El sistema **H.E.R.M.E.S.** utiliza el protocolo MQTT para la comunicación en tiempo real entre el GCS (PC) y el Robot. Necesitas instalar un "Broker" (servidor) MQTT.

Recomendamos **Eclipse Mosquitto**.

## 1. Instalación
1. Descarga el instalador para Windows desde [mosquitto.org/download](https://mosquitto.org/download/).
2. Instala el programa.
   - **IMPORTANTE**: Asegúrate de que se instale como Servicio de Windows (suele ser la opción por defecto).

## 2. Configuración
Para que el robot pueda conectarse desde otra IP (WiFi), necesitamos editar el archivo `mosquitto.conf`.

1. Ve a la carpeta de instalación (generalmente `C:\Program Files\mosquitto`).
2. Abre el archivo `mosquitto.conf` con un editor de texto (como Administrador).
3. Agrega estas líneas al final del archivo para permitir conexiones externas y anónimas (o configura usuarios si prefieres seguridad):

```conf
listener 1883
allow_anonymous true
```

> **Nota de Seguridad**: `allow_anonymous true` es fácil para empezar, pero en producción deberías usar `false` y configurar usuarios/contraseñas.

## 3. Reiniciar el Servicio
Cada vez que cambies el archivo `.conf`, debes reiniciar Mosquitto.
1. Abre **PowerShell** como Administrador.
2. Ejecuta:
   ```powershell
   Restart-Service mosquitto
   ```
3. Verifica que funciona:
   ```powershell
   netstat -an | findstr 1883
   ```
   Deberías ver una línea que dice `LISTENING`.

## 4. Probando la Conexión
Puedes usar nuestro script de prueba incluido:
```powershell
./fix_mqtt.ps1
```
O simplemente iniciar el GCS (`python main.py`) y ver si dice "Conectado al Broker".
