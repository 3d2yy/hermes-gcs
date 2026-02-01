from dash import html, dcc
import dash_mantine_components as dmc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS

def view_radar():
    return html.Div(style={"display": "flex", "gap": "20px", "height": "100%"}, children=[
        html.Div(style={"flex": "1"}, className="gcs-card", children=[
            html.Div(style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "marginBottom": "12px"}, children=[
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px"}, children=[
                    DashIconify(icon="mdi:radar", width=24, color=COLORS['accent_primary']),
                    html.Span("SISTEMA LIDAR / RADAR", className="metric-label"),
                ]),
                dmc.Badge(id="radar-status", color="green", variant="dot", children="ACTIVO"),
            ]),
            dcc.Graph(id="graph-radar", config={"displayModeBar": False}, style={"height": "calc(100% - 50px)"}),
        ]),
        html.Div(style={"width": "280px", "display": "flex", "flexDirection": "column", "gap": "16px"}, children=[
            html.Div(className="gcs-card", children=[
                html.Span("DISTANCIA MÍNIMA", className="metric-label"),
                html.Div(style={"display": "flex", "alignItems": "baseline", "gap": "4px", "marginTop": "8px"}, children=[
                    html.Span(id="radar-min-distance", className="metric-value", style={"fontSize": "2rem"}),
                    html.Span("m", style={"color": COLORS['text_secondary']}),
                ]),
                html.Div(id="radar-min-angle", style={"color": COLORS['text_secondary'], "marginTop": "4px"}),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("OBSTÁCULOS DETECTADOS", className="metric-label"),
                html.Div(id="obstacle-count", style={"marginTop": "12px"}),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("CONFIGURACIÓN", className="metric-label"),
                html.Div(style={"marginTop": "12px"}, children=[
                    html.Div("Alcance Máximo", style={"color": COLORS['text_secondary'], "marginBottom": "4px"}),
                    dmc.Slider(id="radar-range", value=5, min=1, max=10, step=1, 
                               marks=[{"value": i, "label": f"{i}m"} for i in [1, 5, 10]], color="teal"),
                ]),
            ]),
        ]),
    ])
