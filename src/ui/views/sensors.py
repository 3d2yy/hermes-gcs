from dash import html, dcc
from dash_iconify import DashIconify
from src.ui.app_layout import COLORS

def create_metric_card(title, value_id, unit, icon, color=None):
    color = color or COLORS['accent_primary']
    return html.Div(className="gcs-card", style={"minWidth": "140px"}, children=[
        html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px", "marginBottom": "8px"}, children=[
            DashIconify(icon=icon, width=20, color=color),
            html.Span(title, className="metric-label"),
        ]),
        html.Div(style={"display": "flex", "alignItems": "baseline", "gap": "4px"}, children=[
            html.Span(id=value_id, className="metric-value", style={"color": color}),
            html.Span(unit, style={"color": COLORS['text_muted'], "fontSize": "0.875rem"}),
        ]),
    ])

def view_sensors():
    return html.Div(style={"display": "flex", "flexDirection": "column", "gap": "20px"}, children=[
        html.Div(style={"display": "flex", "gap": "16px", "flexWrap": "wrap"}, children=[
            create_metric_card("Gas MQ-2", "sensor-ppm", "ppm", "mdi:gas-cylinder", COLORS['accent_orange']),
            create_metric_card("CO₂", "sensor-co2", "ppm", "mdi:molecule-co2", COLORS['accent_primary']),
            create_metric_card("Temperatura", "sensor-temp", "°C", "mdi:thermometer", COLORS['accent_warning']),
            create_metric_card("Humedad", "sensor-humidity", "%", "mdi:water-percent", COLORS['accent_secondary']),
            create_metric_card("Voltaje", "sensor-voltage", "V", "mdi:battery-charging", COLORS['accent_primary']),
            create_metric_card("Corriente", "sensor-current", "A", "mdi:current-dc", COLORS['accent_secondary']),
        ]),
        html.Div(style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "16px"}, children=[
            html.Div(className="gcs-card", children=[
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px", "marginBottom": "8px"}, children=[
                    DashIconify(icon="mdi:gas-cylinder", width=20, color=COLORS['accent_orange']),
                    html.Span("CONCENTRACIÓN DE GASES", className="metric-label"),
                ]),
                dcc.Graph(id="graph-gas-history", config={"displayModeBar": False}, style={"height": "250px"}),
            ]),
            html.Div(className="gcs-card", children=[
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px", "marginBottom": "8px"}, children=[
                    DashIconify(icon="mdi:thermometer", width=20, color=COLORS['accent_warning']),
                    html.Span("AMBIENTE", className="metric-label"),
                ]),
                dcc.Graph(id="graph-environment", config={"displayModeBar": False}, style={"height": "250px"}),
            ]),
            html.Div(className="gcs-card", children=[
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "8px", "marginBottom": "8px"}, children=[
                    DashIconify(icon="mdi:battery-charging", width=20, color=COLORS['accent_primary']),
                    html.Span("SISTEMA DE POTENCIA", className="metric-label"),
                ]),
                dcc.Graph(id="graph-power", config={"displayModeBar": False}, style={"height": "250px"}),
            ]),
            html.Div(className="gcs-card", children=[
                html.Span("ESTADÍSTICAS", className="metric-label"),
                html.Div(id="sensor-stats", style={"padding": "16px"}),
            ]),
        ]),
    ])
