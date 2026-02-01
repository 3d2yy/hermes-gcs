from dash import html
import dash_mantine_components as dmc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS

def view_logs():
    return html.Div(className="gcs-card", style={"height": "100%"}, children=[
        html.Div(style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "marginBottom": "16px"}, children=[
            html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px"}, children=[
                DashIconify(icon="mdi:console", width=24, color=COLORS['accent_primary']),
                html.Span("REGISTROS DEL SISTEMA", className="metric-label"),
            ]),
            html.Div(style={"display": "flex", "gap": "8px"}, children=[
                dmc.Button("Exportar", id="btn-export-logs", leftSection=DashIconify(icon="mdi:download"), color="gray", variant="outline", size="sm"),
                dmc.Button("Limpiar", id="btn-clear-logs", leftSection=DashIconify(icon="mdi:delete"), color="red", variant="outline", size="sm"),
            ]),
        ]),
        html.Div(id="log-container", style={
            "height": "calc(100% - 60px)", "overflowY": "auto",
            "background": COLORS['bg_primary'], "borderRadius": "6px", "padding": "12px",
        }),
    ])
