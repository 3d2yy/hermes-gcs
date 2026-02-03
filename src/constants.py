"""
Sistema de constantes centralizadas para H.E.R.M.E.S. GCS
Evita el uso de strings mágicos dispersos en el código.
"""

class ConnectionStatus:
    """Estados posibles de conexión del sistema."""
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING..."
    ONLINE = "ONLINE"
    SIMULATED = "SIMULATED"
    REPLAY = "REPLAY FILE"


class SystemMode:
    """Modos de operación del sistema."""
    INIT = "INIT"
    MQTT = "MQTT"
    SIMULATION = "SIMULACIÓN"
    REPLAY = "REPLAY"
    FINISHED = "FINISHED"
    WAITING = "ESPERANDO"


class AlertLevel:
    """Niveles de alerta del sistema."""
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


class AudioClass:
    """Clasificaciones de audio detectables."""
    SILENCE = "SILENCE"
    SCREAM = "SCREAM"
    BREATHING = "BREATHING"
    VOICE = "VOICE"
    GLASS_BREAK = "GLASS_BREAK"
    UNKNOWN = "UNKNOWN"
    
    # Lista de clases que indican detección de persona
    HUMAN_INDICATORS = [SCREAM, BREATHING, VOICE]
    # Lista de clases de alerta
    ALERT_CLASSES = [SCREAM, BREATHING, VOICE, GLASS_BREAK]


# Tópicos MQTT
class MQTTTopics:
    """Tópicos MQTT utilizados por el sistema."""
    # Sensores
    MQ2_DATA = "iot/sensor/mq2/data"
    MQ2_ALERT = "iot/sensor/mq2/alert"
    ENVIRONMENT = "hermes/sensors/environment"
    POWER = "hermes/sensors/power"
    IMU = "hermes/sensors/imu"
    
    # IA/Audio
    AUDIO = "hermes/ai/audio"
    
    # Posición y navegación
    POSITION = "hermes/position"
    RADAR = "hermes/radar"
    
    # Control
    CONTROL = "hermes/control"
