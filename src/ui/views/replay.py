from dash import html, dcc
import dash_mantine_components as dmc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS

def view_replay():
    return html.Div(style={"display": "flex", "flexDirection": "column", "gap": "20px", "alignItems": "center", "justifyContent": "center", "height": "100%"}, children=[
        DashIconify(icon="mdi:movie-open-play", width=80, color=COLORS['accent_secondary']),
        html.H2("REPRODUCCIÓN DE MISIÓN (FORENSIC MODE)", style={"fontFamily": "'Orbitron', sans-serif"}),
        html.P("Cargue datos históricos de la base de datos `mission_data.db` para análisis post-operativo.", 
               style={"color": COLORS['text_secondary'], "maxWidth": "600px", "textAlign": "center"}),
        
        html.Div(className="gcs-card", style={"padding": "30px", "display": "flex", "flexDirection": "column", "gap": "20px", "minWidth": "500px"}, children=[
            # Progress bar
            html.Div(style={"width": "100%"}, children=[
                html.Div(style={"display": "flex", "justifyContent": "space-between", "marginBottom": "8px"}, children=[
                    html.Span(id="replay-time-current", children="00:00", style={"fontFamily": "monospace", "color": COLORS['accent_primary']}),
                    html.Span(id="replay-time-total", children="00:00", style={"fontFamily": "monospace", "color": COLORS['text_secondary']}),
                ]),
                dmc.Progress(id="replay-progress", value=0, color="cyan", size="lg", radius="xl"),
            ]),
            
            # Control buttons
            html.Div(style={"display": "flex", "gap": "20px", "alignItems": "center", "justifyContent": "center"}, children=[
                dmc.Button("Cargar Datos", id="btn-replay-load", leftSection=DashIconify(icon="mdi:database-import"), color="blue", variant="outline"),
                html.Div(style={"width": "2px", "height": "40px", "background": COLORS['border']}),
                dmc.Button(DashIconify(icon="mdi:play"), id="btn-replay-play", color="green", size="lg"),
                dmc.Button(DashIconify(icon="mdi:pause"), id="btn-replay-pause", color="yellow", size="lg"),
                dmc.Button(DashIconify(icon="mdi:stop"), id="btn-replay-stop", color="red", size="lg"),
            ]),
        ]),
        
        html.Div(id="replay-status-text", style={"marginTop": "20px", "fontFamily": "monospace", "color": COLORS['accent_primary']})
    ])
