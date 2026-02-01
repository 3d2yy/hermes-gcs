# 🦅 H.E.R.M.E.S. Ground Control Station (GCS) v2.0

**Sistema de Monitoreo y Teleoperación para Robots de Rescate en Espacios Confinados.**

![Status](https://img.shields.io/badge/Status-Stable-success)
![Python](https://img.shields.io/badge/Python-3.13-blue)
![Dash](https://img.shields.io/badge/Dash-2.0-orange)

La **GCS H.E.R.M.E.S. v2.0** es una interfaz de comando avanzada diseñada para operar robots exploradores en entornos peligrosos. Permite la visualización en tiempo real de datos de sensores, mapeo 3D de gases peligrosos, y análisis forense de misiones.

---

## 🚀 Características Principales

*   **📊 Telemetría en Tiempo Real**: Visualización de PPM (MQ-2), CO2, temperatura, humedad y estado de batería.
*   **🗺️ Mapeo de Gases 3D**: Generación dinámica de mapas de calor (Heatmaps) interpolados sobre el terreno explorado.
*   **🎮 Teleoperación**: Control de movimiento, luces y feed de video con baja latencia.
*   **🔁 Mission Replay (Forense)**: Sistema de grabación y reproducción de misiones pasadas para análisis post-operativo.
*   **📡 Arquitectura Modular**: Separación limpia entre UI, Servicios (MQTT/DB) y Estado Global.

---

## 🛠️ Instalación

1.  **Clonar el repositorio:**
    ```bash
    git clone https://github.com/3d2yy/hermes-gcs.git
    cd hermes-gcs
    ```

2.  **Instalar dependencias:**
    ```bash
    pip install dash dash-mantine-components dash-iconify plotly paho-mqtt opencv-python scipy flask requests
    ```

3.  **Configuración:**
    *   Copia `config.example.json` a `config.json`.
    *   Edita las IPs de tu robot y broker MQTT.

---

## ▶️ Ejecución

Para iniciar la estación de control:

```bash
python main.py
```

Accede a la interfaz en tu navegador: `http://127.0.0.1:8050`

---

## 📂 Estructura del Proyecto

```text
hermes_gcs/
 ├── main.py                 # Punto de entrada de la aplicación
 ├── config.json             # Configuración (Ignorado por git)
 ├── assets/                 # Estilos CSS y recursos estáticos
 └── src/
      ├── state.py           # Estado Global (Singleton)
      ├── services/          # Comunicación y Lógica de Fondo (MQTT, DB, Replay)
      └── ui/                # Componentes Visuales y Vistas
```

---

## 🤝 Contribución

1.  Haz un Fork del proyecto.
2.  Crea tu rama de funcionalidades (`git checkout -b feature/AmazingFeature`).
3.  Haz Commit de tus cambios (`git commit -m 'Add some AmazingFeature'`).
4.  Haz Push a la rama (`git push origin feature/AmazingFeature`).
5.  Abre un Pull Request.

---

**Desarrollado para el Proyecto H.E.R.M.E.S.**
