# H.E.R.M.E.S. - Ground Control Station & Robot Firmware
**Hostile Environment Reconnaissance - Micro-controlled Execution System**

H.E.R.M.E.S. es un sistema avanzado de exploración robótica diseñado para monitoreo ambiental y reconocimiento en entornos peligrosos. Combina un robot basado en ESP32 con una Estación de Control en Tierra (GCS) moderna construida en Python.

## ✨ Características Principales
- **Telepresencia**: Video en tiempo real vía ESP32-CAM.
- **Sensores Avanzados**: Detección de gases (MQ-2), CO2, temperatura y humedad (SCD30).
- **Navegación Estable**: Control PID asistido por giroscopio (MPU6050).
- **Control Total**: Interfaz Dash responsiva para teleoperación manual.
- **Seguridad**: Protocolo MQTT v2.2 con autenticación y "watchdog" de seguridad.

---

## 🚀 Guía de Inicio Rápido (GCS)

### 1. Requisitos Previos
- Python 3.9+
- Servidor MQTT (Mosquitto) corriendo.

### 2. Instalación
```bash
# Instalar dependencias
pip install -r requirements.txt
```

### 3. Ejecución
```bash
python main.py
```
Abre tu navegador en `http://localhost:8050`.

---

## 🔧 Configuración del Hardware

Para construir y configurar tu propio robot H.E.R.M.E.S., sigue estas guías detalladas:

1. **[Configuración del Servidor MQTT](setup/GUIDE_MOSQUITTO.md)**: El corazón de las comunicaciones.
2. **[Configuración del Robot (ESP32)](setup/GUIDE_ESP32.md)**: Flasheo de MicroPython y carga de código.
3. **[Configuración de la Cámara (ESP32-CAM)](setup/GUIDE_ESP32_CAM.md)**: Programación del módulo de video.

### ⚠️ Notas de Seguridad para Desarrollo
Este repositorio no incluye credenciales reales.
- **ESP32 Firmware**: Debes crear `firmware/secrets.py` basado en tus claves reales (ver guía).
- **ESP32-CAM**: Debes editar `ESP32_CAM.ino` antes de subirlo.
- **GCS**: Edita `src/config.py` o crea un `config.json` local.

---

## 📁 Estructura del Proyecto
- **/firmware**: Código para el ESP32 (MicroPython).
  - `main.py`: Cerebro del robot.
  - `drivers.py`: Controladores de hardware.
  - `secrets.py`: (GitIgnored) Tus claves privadas.
- **/src**: Código de la Estación de Control (GCS).
  - `/services`: Lógica de MQTT y simulaciones.
  - `/ui`: Interfaz gráfica (Dash).
- **/camera_firmware**: Código C++ para ESP32-CAM.
- **/setup**: Guías y scripts de configuración.
