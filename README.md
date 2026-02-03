# H.E.R.M.E.S. - Ground Control Station & Robot Firmware
**Hostile Environment Reconnaissance - Micro-controlled Execution System**

H.E.R.M.E.S. es un sistema avanzado de exploración y monitoreo ambiental basado en ESP32 y una estación de control en tierra (GCS) desarrollada en Python/Dash. Diseñado para misiones de reconocimiento en entornos peligrosos.

## ✨ Características Principales
- **Teleoperación Multi-modo**: Control manual, diferencial y PID.
- **Detección de Amenazas**: Sensor de gas/humo MQ-2 con alertas en tiempo real.
- **Monitoreo Ambiental**: Sensor SCD30 para CO2, humedad y temperatura.
- **Control de Estabilidad**: Algoritmos PID integrados con el MPU6050 para navegación recta.
- **Seguridad Robusta**: Autenticación MQTT y versionado de protocolo (v2.2).
- **Logs Inteligentes**: Sistema de logging estructurado con iconos para facilitar el debug.

## 📁 Estructura del Proyecto
- **/firmware**: Código para el ESP32 (MicroPython).
  - `main.py`: Loop principal de control asíncrono con `uasyncio`.
  - `drivers.py`: Abstracción de hardware para sensores y motores.
  - `config.py`: Configuración unificada de pines, red y seguridad.
  - `pid.py`: Implementación del controlador PID.
  - `MPU6050.py`: Driver para el sensor inercial.
- **/src**: Código de la Estación de Control (GCS).
  - `/services`: Servicios de comunicación MQTT y procesamiento de datos.
  - `/ui`: Interfaz gráfica moderna basada en Dash/Plotly.
  - `main.py`: Punto de entrada del servidor GCS.

## 🚀 Versión 2.2 - Mejoras de Seguridad y Estabilidad
La versión actual introduce:
- **MQTT Auth**: Seguridad por usuario/contraseña obligatoria.
- **Recalibración Remota**: Comando `CALIBRATE` para resetear el IMU vía aire.
- **Optimizaciones Asíncronas**: Mejor rendimiento en el control de motores (50Hz) y lectura de sensores.
- **Fusión de Datos**: Los sensores MQ-2 y SCD30 trabajan coordinados para identificar tipos de amenazas.

## 🔧 Instalación
Consulte la [GUÍA DE INSTALACIÓN](INSTRUCCIONES_INSTALACION.md) para configurar su hardware y software.

---

## 📈 Roadmap (Innovación Futura)
- [ ] **Data Science**: Mapas de calor (Heatmaps) de concentraciones de gas en tiempo real.
- [ ] **Edge AI**: Detección de tipos de terreno mediante vibraciones (TinyML).
- [ ] **Modularidad**: Integración con Raspberry Pi 4 y ROS2.
- [ ] **Visión**: Superposición de telemetría (OSD) sobre el video de la cámara.

---
*Desarrollado para misiones de exploración avanzada - 2026*
