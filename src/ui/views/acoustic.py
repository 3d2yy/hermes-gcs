from dash import html, dcc
import dash_mantine_components as dmc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS

def view_acoustic():
    return html.Div(style={"display": "flex", "flexDirection": "column", "gap": "20px"}, children=[
        html.Div(style={"display": "grid", "gridTemplateColumns": "repeat(4, 1fr)", "gap": "16px"}, children=[
            html.Div(className="gcs-card", style={"textAlign": "center"}, children=[
                DashIconify(icon="mdi:account-voice", width=40, color=COLORS['accent_secondary']),
                html.Div("CLASIFICACIÓN", className="metric-label", style={"marginTop": "8px"}),
                html.Div(id="acoustic-current-class", className="metric-value", style={"fontSize": "1.5rem"}),
            ]),
            html.Div(className="gcs-card", style={"textAlign": "center"}, children=[
                DashIconify(icon="mdi:percent", width=40, color=COLORS['accent_primary']),
                html.Div("CONFIANZA", className="metric-label", style={"marginTop": "8px"}),
                html.Div(id="acoustic-confidence", className="metric-value", style={"fontSize": "1.5rem"}),
            ]),
            html.Div(className="gcs-card", style={"textAlign": "center"}, children=[
                DashIconify(icon="mdi:compass", width=40, color=COLORS['accent_warning']),
                html.Div("DIRECCIÓN", className="metric-label", style={"marginTop": "8px"}),
                html.Div(id="acoustic-direction", className="metric-value", style={"fontSize": "1.5rem"}),
            ]),
            html.Div(className="gcs-card", style={"textAlign": "center"}, children=[
                DashIconify(icon="mdi:counter", width=40, color=COLORS['accent_danger']),
                html.Div("DETECCIONES", className="metric-label", style={"marginTop": "8px"}),
                html.Div(id="acoustic-count", className="metric-value", style={"fontSize": "1.5rem"}),
            ]),
        ]),
        html.Div(style={"display": "grid", "gridTemplateColumns": "2fr 1fr", "gap": "16px"}, children=[
            html.Div(className="gcs-card", children=[
                html.Span("HISTÓRICO DE CONFIANZA", className="metric-label"),
                dcc.Graph(id="graph-audio-confidence", config={"displayModeBar": False}, style={"height": "300px"}),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("DISTRIBUCIÓN DE CLASES", className="metric-label"),
                dcc.Graph(id="graph-audio-classes", config={"displayModeBar": False}, style={"height": "300px"}),
            ]),
        ]),
        html.Div(className="gcs-card", children=[
            html.Span("REGISTRO DE DETECCIONES", className="metric-label"),
            html.Div(id="acoustic-detection-log", style={"maxHeight": "200px", "overflowY": "auto", "marginTop": "12px"}),
        ]),
    ])
