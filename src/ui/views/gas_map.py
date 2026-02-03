from dash import html, dcc
import dash_mantine_components as dmc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS

def view_gas_map():
    return html.Div(style={"display": "flex", "gap": "20px", "height": "100%"}, children=[
        html.Div(style={"flex": "1", "display": "flex", "flexDirection": "column"}, className="gcs-card", children=[
            html.Div(style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "marginBottom": "12px"}, children=[
                html.Div([
                    html.Span("MAPA DE DETECCIÓN DE GASES", className="metric-label"),
                    html.Div(id="robot-position-display", style={"color": COLORS['accent_primary'], "marginTop": "4px"}),
                ]),
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px"}, children=[
                    html.Span("Lectura Actual:", style={"color": COLORS['text_secondary']}),
                    html.Span(id="current-ppm-reading", className="metric-value", style={"fontSize": "1.25rem", "color": COLORS['accent_orange']}),
                ]),
            ]),
            dcc.Graph(id="gas-heatmap", config={"displayModeBar": False}, style={"flex": "1", "minHeight": "0"}),
        ]),
        html.Div(style={"width": "280px", "display": "flex", "flexDirection": "column", "gap": "16px"}, children=[
            html.Div(className="gcs-card", children=[
                html.Span("CONTROLES", className="metric-label"),
                html.Div(style={"marginTop": "16px"}, children=[
                    dmc.SegmentedControl(id="map-view-mode", value="2d", 
                        data=[{"value": "2d", "label": "2D Plano"}, {"value": "3d", "label": "3D Terreno"}],
                        color="teal", fullWidth=True, style={"marginBottom": "16px"}),
                    dmc.Switch(id="show-grid", label="Mostrar Cuadrícula", checked=True, color="teal", size="sm"),
                    dmc.Switch(id="show-heatmap", label="Mostrar Mapa de Calor", checked=True, color="teal", size="sm", style={"marginTop": "8px"}),
                    dmc.Switch(id="show-path", label="Mostrar Trayectoria", checked=True, color="teal", size="sm", style={"marginTop": "8px"}),
                ]),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("LEYENDA", className="metric-label"),
                html.Div(style={"marginTop": "12px"}, children=[
                    html.Div(style={"height": "20px", "background": "linear-gradient(90deg, #00ff88, #ffd60a, #ff4757)", "borderRadius": "4px", "marginBottom": "8px"}),
                    html.Div(style={"display": "flex", "justifyContent": "space-between"}, children=[
                        html.Span("300 ppm", style={"fontSize": "0.75rem", "color": COLORS['text_secondary']}),
                        html.Span("10000 ppm", style={"fontSize": "0.75rem", "color": COLORS['text_secondary']}),
                    ]),
                ]),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("ESTADÍSTICAS", className="metric-label"),
                html.Div(id="gas-map-stats", style={"marginTop": "12px"}),
            ]),
            dmc.Button("Limpiar Datos", id="btn-clear-gas-map", leftSection=DashIconify(icon="mdi:delete"), color="red", variant="outline", fullWidth=True),
        ]),
    ])
