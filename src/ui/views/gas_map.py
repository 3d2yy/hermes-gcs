from dash import html, dcc
import dash_mantine_components as dmc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS
try:
    from src.state import state
except ImportError:
    # If state isn't directly importable here due to some reason (circular?), we rely on callbacks
    # But callbacks are in main.py, so this file is mostly layout.
    # However, view_gas_map might need state if it renders initial state?
    # Actually main.py passes state to callbacks. view_gas_map function usually just returns layout.
    # checking original view_gas_map content: it used 'state' inside the function?
    pass

# Wait, view_gas_map used 'state' in update_gas_map callback, which is now in main.py.
# The layout function `view_gas_map` didn't seem to use state directly in the layout construction
# EXCEPT for the graph ID and static structure.
# Let's check the original content I wrote in step 98.
# It accessed `state.robot_position` etc? No, wait.
# Step 98: `view_gas_map` function used `state.gas_map_points`? No.
# It defines layout. The dynamic content is updated by callbacks.
# Ah, Step 14 callback `update_gas_map` is in `main.py` now.
# So `view_gas_map` just returns static layout with IDs.
# Does `view_gas_map` need state?
# Step 98 code: 
# def view_gas_map():
#    ... children=[ ... dcc.Graph(id="gas-heatmap" ...) ]
# It does NOT use `state`. It just uses `COLORS`.
# BUT wait, the `update_gas_map` callback IS in main.py.
# So `view_gas_map.py` only needs `COLORS`.

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
