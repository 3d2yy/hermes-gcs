from dash import html, dcc
import dash_mantine_components as dmc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS
from src.config import CONFIG

def create_control_pad():
    return html.Div(style={
        "display": "grid",
        "gridTemplateColumns": "60px 60px 60px",
        "gridTemplateRows": "60px 60px 60px",
        "gap": "8px",
        "justifyContent": "center",
    }, children=[
        html.Div(),
        html.Button(DashIconify(icon="mdi:arrow-up-bold", width=28), id="btn-forward", className="control-button"),
        html.Div(),
        html.Button(DashIconify(icon="mdi:arrow-left-bold", width=28), id="btn-left", className="control-button"),
        html.Button(DashIconify(icon="mdi:stop-circle", width=28, color=COLORS['accent_danger']), id="btn-stop", 
                    className="control-button", style={"borderColor": COLORS['accent_danger']}),
        html.Button(DashIconify(icon="mdi:arrow-right-bold", width=28), id="btn-right", className="control-button"),
        html.Div(),
        html.Button(DashIconify(icon="mdi:arrow-down-bold", width=28), id="btn-backward", className="control-button"),
        html.Div(),
    ])

def view_teleop():
    return html.Div(style={"display": "flex", "gap": "20px", "height": "100%"}, children=[
        html.Div(style={"flex": "1", "display": "flex", "flexDirection": "column", "gap": "16px"}, children=[
            html.Div(style={"display": "flex", "gap": "8px"}, children=[
                dmc.SegmentedControl(id="camera-selector", value="rgb",
                    data=[{"value": "rgb", "label": "RGB"}, {"value": "ir", "label": "IR"}, {"value": "thermal", "label": "Térmica"}],
                    color="teal"),
                html.Div(style={"flex": "1"}),
                dmc.Badge(id="camera-status", color="gray", children="DESCONECTADO"),
            ]),
            html.Div(className="video-container", children=[
                html.Img(id="video-feed", 
                    src=f"http://{CONFIG.get('camera_ip', CONFIG.get('mqtt_broker', '127.0.0.1'))}:{CONFIG['camera_port']}/stream",
                    style={"width": "100%", "height": "100%", "objectFit": "contain"},
                    alt="Esperando señal de video..."),
                html.Div(className="video-overlay"),
                html.Div(style={"position": "absolute", "top": "50%", "left": "50%", "transform": "translate(-50%, -50%)",
                    "width": "60px", "height": "60px", "border": f"2px solid {COLORS['accent_primary']}40", "borderRadius": "50%"}),
            ]),
            html.Div(style={"display": "flex", "gap": "8px"}, children=[
                dmc.Button("Iniciar Cámara", id="btn-start-camera", leftSection=DashIconify(icon="mdi:camera"), color="teal", variant="outline"),
                dmc.Button("Captura", id="btn-snapshot", leftSection=DashIconify(icon="mdi:camera-iris"), color="gray", variant="outline"),
                dmc.Button("Grabar", id="btn-record", leftSection=DashIconify(icon="mdi:record-circle"), color="red", variant="outline"),
            ]),
        ]),
        html.Div(style={"width": "300px", "display": "flex", "flexDirection": "column", "gap": "16px"}, children=[
            html.Div(className="gcs-card", children=[
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px", "marginBottom": "12px"}, children=[
                    DashIconify(icon="mdi:ear-hearing", width=24, color=COLORS['accent_secondary']),
                    html.Span("IA AUDIO", style={"fontFamily": "'Rajdhani', sans-serif", "fontWeight": "600", "letterSpacing": "2px"}),
                ]),
                html.Div(id="audio-class-display", style={
                    "fontFamily": "'Orbitron', monospace", "fontSize": "1.5rem", "fontWeight": "700",
                    "color": COLORS['accent_secondary'], "marginBottom": "8px"}, children="SILENCE"),
                dmc.Progress(id="audio-confidence-bar", value=0, color="cyan", size="lg", radius="xl", striped=True, animated=True),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("TELEMETRÍA RÁPIDA", style={"fontFamily": "'Rajdhani', sans-serif", "fontSize": "0.75rem", "color": COLORS['text_muted'], "letterSpacing": "2px"}),
                html.Div(style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "16px", "marginTop": "12px"}, children=[
                    html.Div([html.Div("GAS PPM", className="metric-label"),
                              html.Div(id="teleop-ppm", className="metric-value", style={"fontSize": "1.5rem", "color": COLORS['accent_orange']})]),
                    html.Div([html.Div("CO₂", className="metric-label"),
                              html.Div(id="teleop-co2", className="metric-value", style={"fontSize": "1.5rem"})]),
                    html.Div([html.Div("TEMP", className="metric-label"),
                              html.Div(id="teleop-temp", className="metric-value", style={"fontSize": "1.5rem", "color": COLORS['accent_warning']})]),
                    html.Div([html.Div("VOLTAJE", className="metric-label"),
                              html.Div(id="teleop-voltage", className="metric-value", style={"fontSize": "1.5rem", "color": COLORS['accent_secondary']})]),
                ]),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("NAVEGACIÓN", style={"fontFamily": "'Rajdhani', sans-serif", "fontSize": "0.75rem", "color": COLORS['text_muted'], "letterSpacing": "2px", "display": "block", "marginBottom": "12px", "textAlign": "center"}),
                create_control_pad(),
                html.Div(style={"marginTop": "16px", "marginBottom": "16px"}, children=[
                    html.Span("LUZ (LED)", className="metric-label", style={"fontSize": "0.7rem", "marginBottom": "8px"}),
                    dmc.Slider(
                        id="led-intensity-slider",
                        min=0,
                        max=255,
                        value=128,
                        step=1,
                        color="yellow",
                        marks=[{"value": 0, "label": "OFF"}, {"value": 255, "label": "MAX"}],
                        size="sm"
                    ),
                    html.Div(id="led-status-text", style={"fontSize": "0.7rem", "marginTop": "4px", "color": "#8b949e", "textAlign": "center"})
                ]),
                html.Div(style={"display": "flex", "justifyContent": "center", "gap": "8px", "marginTop": "12px"}, children=[
                    dmc.Button(DashIconify(icon="mdi:lightbulb-on"), id="btn-lights", color="yellow", variant="outline", size="sm"),
                    dmc.Button(DashIconify(icon="mdi:volume-high"), id="btn-speaker", color="cyan", variant="outline", size="sm"),
                    dmc.Button(DashIconify(icon="mdi:horn"), id="btn-horn", color="orange", variant="outline", size="sm"),
                ]),
            ]),
        ]),
    ])
