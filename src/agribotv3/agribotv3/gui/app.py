#!/usr/bin/env python3
"""agribotv3 web dashboard — nicegui + Nav2.

Run standalone:  python3 -m agribotv3.gui.app
Or via ROS:      ros2 run agribotv3 gui

Opens http://localhost:8080 with:
  • Dark industrial UI with dual-weight plotly grid
  • Plant-cell colouring by batch / state  (plants.csv)
  • Soil Scan: select area → N random waypoints with dwell
  • Plant Region: select area → plant-cell centres as waypoints
  • Camera feed (MJPEG, 640×480)
  • Ultrasonic sensor cones on map + bar readouts
  • Manual WASD drive  (on-screen buttons + keyboard)
  • Nav2 status / E-STOP / cancel
  • Seed dropdown from seeds.csv
  • Activity log
"""
from __future__ import annotations

import asyncio
import math
import os
import random
import time
from typing import Optional

import numpy as np

from nicegui import app, ui
from starlette.responses import StreamingResponse

from .config import (
    WORLD_MIN_X, WORLD_MAX_X, WORLD_MIN_Y, WORLD_MAX_Y,
    GRID_CONFIG, CELL_W, CELL_H, STATE_COLOURS,
    ROBOT_LENGTH, ROBOT_WIDTH,
    SENSOR_OFFSETS, US_FOV, US_MAX_RANGE,
    BATCH_COLOURS, get_batch_colour, PLANT_STATE_COLOURS,
    PLANT_CELL_SIZE, SOIL_SCAN_DWELL, SOIL_SCAN_N_DEFAULT,
    load_plants_csv, load_seeds_csv,
)
from .database import AgriBotDB
from .controller import RosController, world_to_grid, grid_to_world


# ═══════════════════════════════════════════════════════════════
#  Global singletons
# ═══════════════════════════════════════════════════════════════
ctrl: RosController | None = None
db:   AgriBotDB    | None = None


def _ensure_singletons():
    global ctrl, db
    if ctrl is None:
        ctrl = RosController()
        ctrl.start()
    if db is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            data_dir = os.path.join(get_package_share_directory('agribotv3'), 'data')
        except Exception:
            data_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'data')
        db = AgriBotDB(data_dir)


# ═══════════════════════════════════════════════════════════════
#  MJPEG camera endpoint
# ═══════════════════════════════════════════════════════════════
@app.get('/video_feed')
async def _video_feed():
    import cv2

    async def _gen():
        while True:
            try:
                if ctrl and ctrl.state.cam_data and ctrl.state.cam_w > 0:
                    st = ctrl.state
                    w, h, enc = st.cam_w, st.cam_h, st.cam_encoding
                    raw = st.cam_data
                    if raw and len(raw) > 0:
                        ch = 4 if 'a' in enc else 3
                        if len(raw) == h * w * ch:
                            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, ch)
                            if ch == 4:
                                arr = arr[:, :, :3]
                            if enc.startswith('rgb'):
                                arr = arr[:, :, ::-1]
                            _, jpg = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                                   + jpg.tobytes() + b'\r\n')
            except Exception:
                pass
            await asyncio.sleep(0.05)

    return StreamingResponse(_gen(), media_type='multipart/x-mixed-replace; boundary=frame')


# ═══════════════════════════════════════════════════════════════
#  Map figure builder
# ═══════════════════════════════════════════════════════════════
def _build_map_fig(
    robot_x: float, robot_y: float, robot_yaw: float,
    us_vals: dict,
    path_pts: list,
    goal_x: float, goal_y: float, has_goal: bool,
    plants: list[dict],
    selection_area: tuple | None,       # (x0, y0, x1, y1) world coords
    mission_waypoints: list[tuple],
    mission_wp_idx: int,
):
    import plotly.graph_objects as go

    fig = go.Figure()
    fig.update_layout(
        xaxis=dict(
            range=[WORLD_MIN_X, WORLD_MAX_X],
            constrain='domain', scaleanchor='y', scaleratio=1,
            showgrid=False, zeroline=False,
            tickfont=dict(color='#4fc3f7', size=10),
            title=dict(text='X (m)', font=dict(color='#4fc3f7', size=11)),
        ),
        yaxis=dict(
            range=[WORLD_MIN_Y, WORLD_MAX_Y],
            constrain='domain',
            showgrid=False, zeroline=False,
            tickfont=dict(color='#4fc3f7', size=10),
            title=dict(text='Y (m)', font=dict(color='#4fc3f7', size=11)),
        ),
        margin=dict(l=42, r=8, t=10, b=42),
        plot_bgcolor='#111827',
        paper_bgcolor='#0d1117',
        showlegend=False,
        dragmode='pan',
        height=500,
    )

    # ── Dual-weight grid (0.5 m minor / 1 m major) ──────────
    x = WORLD_MIN_X
    while x <= WORLD_MAX_X + 1e-6:
        is_major = abs(round(x) - x) < 1e-4
        fig.add_shape(type='line', x0=x, x1=x, y0=WORLD_MIN_Y, y1=WORLD_MAX_Y,
                      line=dict(color='#1e3a5f' if is_major else '#121c2e',
                                width=0.9 if is_major else 0.35))
        x = round(x + 0.5, 2)
    y = WORLD_MIN_Y
    while y <= WORLD_MAX_Y + 1e-6:
        is_major = abs(round(y) - y) < 1e-4
        fig.add_shape(type='line', x0=WORLD_MIN_X, x1=WORLD_MAX_X, y0=y, y1=y,
                      line=dict(color='#1e3a5f' if is_major else '#121c2e',
                                width=0.9 if is_major else 0.35))
        y = round(y + 0.5, 2)

    # ── Plant cells from CSV ─────────────────────────────────
    plotted: set = set()
    for p in plants:
        px_w, py_w = float(p['x']), float(p['y'])
        cx = math.floor(px_w / PLANT_CELL_SIZE) * PLANT_CELL_SIZE + PLANT_CELL_SIZE / 2
        cy = math.floor(py_w / PLANT_CELL_SIZE) * PLANT_CELL_SIZE + PLANT_CELL_SIZE / 2
        key = (round(cx, 3), round(cy, 3))
        if key in plotted:
            continue
        plotted.add(key)
        s = p.get('state', 'not_planted')
        if s == 'detected':
            fill = PLANT_STATE_COLOURS['detected']
        elif s == 'not_planted':
            fill = PLANT_STATE_COLOURS['not_planted']
        else:
            fill = get_batch_colour(p.get('batch_id', ''))
        hs = PLANT_CELL_SIZE / 2
        # shaded cell
        fig.add_shape(type='rect',
                      x0=cx - hs, y0=cy - hs, x1=cx + hs, y1=cy + hs,
                      line=dict(color=fill, width=1.5),
                      fillcolor=fill, opacity=0.45)
        # centre square marker
        fig.add_trace(go.Scatter(
            x=[cx], y=[cy], mode='markers',
            marker=dict(size=8, color=fill, symbol='square',
                        line=dict(color='white', width=1)),
            hovertext=(f"<b>{p.get('plant_id', '')}</b>  batch: {p.get('batch_id', '')}"
                       f"<br>state: {s}  seed: {p.get('seed_id', '')}"),
            hoverinfo='text', showlegend=False,
        ))

    # ── DB grid cells (legacy 3×3 state grid) ────────────────
    cells = db.get_all_cells() if db else []
    for gx, gy, cell_state, *_ in cells:
        colour = STATE_COLOURS.get(cell_state, STATE_COLOURS['empty'])
        x0 = GRID_CONFIG['min_x'] + gx * CELL_W
        y0 = GRID_CONFIG['min_y'] + gy * CELL_H
        fig.add_shape(type='rect', x0=x0, y0=y0,
                      x1=x0 + CELL_W, y1=y0 + CELL_H,
                      line=dict(color='#556', width=1),
                      fillcolor=colour, opacity=0.25)

    # ── Area-selection rectangle ─────────────────────────────
    if selection_area:
        ax0, ay0, ax1, ay1 = selection_area
        fig.add_shape(type='rect', x0=ax0, y0=ay0, x1=ax1, y1=ay1,
                      line=dict(color='#00dcff', width=1.5, dash='dot'),
                      fillcolor='rgba(0,220,255,0.06)')

    # ── Nav2 planned path ────────────────────────────────────
    if len(path_pts) > 1:
        xs, ys = zip(*path_pts)
        fig.add_trace(go.Scatter(
            x=list(xs), y=list(ys), mode='lines+markers',
            line=dict(color='#00e676', width=1.5, dash='dot'),
            marker=dict(size=3, color='#00e676'),
            hoverinfo='skip', showlegend=False,
        ))

    # ── Mission waypoints ────────────────────────────────────
    for i, (wx, wy) in enumerate(mission_waypoints):
        done   = i < mission_wp_idx
        active = i == mission_wp_idx
        colour = '#555' if done else ('#00ff88' if active else '#00bcd4')
        fig.add_trace(go.Scatter(
            x=[wx], y=[wy], mode='markers+text',
            marker=dict(size=11 if active else 8, color=colour,
                        line=dict(color='white', width=2 if active else 1)),
            text=[f'WP{i + 1}'], textposition='top right',
            textfont=dict(color=colour, size=10),
            hovertext=f'WP{i + 1}  ({wx:.2f}, {wy:.2f})',
            hoverinfo='text', showlegend=False,
        ))

    # ── Nav goal marker ──────────────────────────────────────
    if has_goal:
        fig.add_trace(go.Scatter(
            x=[goal_x], y=[goal_y], mode='markers',
            marker=dict(symbol='x-thin', size=18, color='#ff4444',
                        line=dict(width=3, color='#ff4444')),
            hovertext=f'Goal ({goal_x:.1f}, {goal_y:.1f})',
            hoverinfo='text', showlegend=False,
        ))

    # ── Ultrasonic cones ─────────────────────────────────────
    cone_colours = [
        'rgba(255,140,0,0.22)', 'rgba(255,80,0,0.22)',
        'rgba(255,200,0,0.22)', 'rgba(200,110,0,0.22)',
    ]
    for i, (key, offs) in enumerate(SENSOR_OFFSETS.items()):
        d = us_vals.get(key, US_MAX_RANGE)
        if d is None or not math.isfinite(d):
            d = US_MAX_RANGE
        d = min(float(d), US_MAX_RANGE)

        cos_r = math.cos(robot_yaw)
        sin_r = math.sin(robot_yaw)
        sx = robot_x + offs['x'] * cos_r - offs['y'] * sin_r
        sy = robot_y + offs['x'] * sin_r + offs['y'] * cos_r
        sensor_yaw = robot_yaw + offs['yaw']

        n = 14
        angles = [sensor_yaw - US_FOV / 2 + j * US_FOV / n for j in range(n + 1)]
        arc_x = [sx] + [sx + d * math.cos(a) for a in angles] + [sx]
        arc_y = [sy] + [sy + d * math.sin(a) for a in angles] + [sy]

        cc = cone_colours[i % len(cone_colours)]
        fig.add_trace(go.Scatter(
            x=arc_x, y=arc_y, mode='lines',
            fill='toself', fillcolor=cc,
            line=dict(color=cc.replace('0.22', '0.55'), width=0.7),
            hoverinfo='skip', showlegend=False,
        ))

    # ── Robot body ───────────────────────────────────────────
    hl, hw = ROBOT_LENGTH / 2, ROBOT_WIDTH / 2
    ca, sa = math.cos(robot_yaw), math.sin(robot_yaw)
    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)]
    rxs = [robot_x + cx * ca - cy * sa for cx, cy in corners]
    rys = [robot_y + cx * sa + cy * ca for cx, cy in corners]
    fig.add_trace(go.Scatter(
        x=rxs, y=rys, mode='lines', fill='toself',
        line=dict(color='#4fc3f7', width=2),
        fillcolor='rgba(21,101,192,0.45)',
        hoverinfo='skip', showlegend=False,
    ))
    ax_tip = robot_x + hl * ca
    ay_tip = robot_y + hl * sa
    fig.add_trace(go.Scatter(
        x=[robot_x, ax_tip], y=[robot_y, ay_tip], mode='lines+markers',
        marker=dict(symbol='arrow', size=13, angleref='previous', color='#FFD700'),
        line=dict(color='#FFD700', width=2),
        hoverinfo='skip', showlegend=False,
    ))
    fig.add_trace(go.Scatter(
        x=[robot_x], y=[robot_y], mode='markers',
        marker=dict(size=6, color='#ff4444'),
        hovertext=f'Robot  x={robot_x:.2f}  y={robot_y:.2f}  yaw={math.degrees(robot_yaw):.0f}°',
        hoverinfo='text', showlegend=False,
    ))

    # Invisible click-capture layer
    _cx = [WORLD_MIN_X + i * (WORLD_MAX_X - WORLD_MIN_X) / 49 for i in range(50)]
    _cy = [WORLD_MIN_Y + i * (WORLD_MAX_Y - WORLD_MIN_Y) / 49 for i in range(50)]
    fig.add_trace(go.Heatmap(
        z=[[0] * 50 for _ in range(50)], x=_cx, y=_cy,
        showscale=False, opacity=0.01,
        colorscale=[[0, '#111827'], [1, '#111827']],
        hovertemplate='x=%{x:.2f}  y=%{y:.2f}<extra></extra>',
    ))

    return fig


# ═══════════════════════════════════════════════════════════════
#  Page
# ═══════════════════════════════════════════════════════════════
@ui.page('/')
def index():
    _ensure_singletons()
    assert ctrl is not None and db is not None

    st = ctrl.state

    # Static CSV data
    plants = load_plants_csv()
    seeds  = load_seeds_csv()
    seed_options = ({s['seed_id']: f"{s['seed_id']} – {s['name']}" for s in seeds}
                    if seeds else {'': 'No seeds found'})

    # Per-page state
    logs: list[str]  = []
    click_mode       = {'mode': None}   # None | 'soil_scan_area' | 'plant_area'
    area_corner      = {'pt': None}
    pending_area: dict = {}

    def _log(msg: str):
        ts = time.strftime('%H:%M:%S')
        logs.insert(0, f'[{ts}] {msg}')
        if len(logs) > 200:
            logs.pop()
        _refresh_log()

    # ── Global styles ────────────────────────────────────────
    ui.add_head_html("""
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;500;600;700&display=swap" rel="stylesheet">
    <style>
      :root {
        --bg:#0d1117; --panel:#111827; --border:#1e3a5f;
        --accent:#00d4ff; --accent2:#00e676; --warn:#ff6b35; --danger:#ff2244;
        --text:#c8d8e8; --dim:#4a6080;
        --mono:'Share Tech Mono',monospace; --sans:'Rajdhani',sans-serif;
      }
      body,.nicegui-content{background:var(--bg)!important;color:var(--text)!important}
      *{font-family:var(--sans)}
      .q-card{background:var(--panel)!important;border:1px solid var(--border)!important;border-radius:10px!important}
      .q-header{background:#080e18!important;border-bottom:1px solid var(--border)!important}
      .ptitle{font-size:10px;font-weight:700;letter-spacing:2px;text-transform:uppercase;color:var(--accent);padding:2px 0 6px;display:block}
      .badge{display:inline-block;padding:2px 10px;border-radius:20px;font-family:var(--mono);font-size:11px;font-weight:600;letter-spacing:1px}
      .b-idle  {background:rgba(0,212,255,.10);color:var(--accent);border:1px solid var(--accent)}
      .b-active{background:rgba(0,230,118,.10);color:var(--accent2);border:1px solid var(--accent2);animation:pulse 1.5s infinite}
      .b-warn  {background:rgba(255,107,53,.15);color:var(--warn);border:1px solid var(--warn)}
      .b-danger{background:rgba(255,34,68,.20);color:var(--danger);border:1px solid var(--danger);animation:pulse .8s infinite}
      @keyframes pulse{0%,100%{opacity:1}50%{opacity:.55}}
      .coord{font-family:var(--mono);font-size:14px;color:var(--accent2)}
      .log-row{font-family:var(--mono);font-size:11px;color:var(--dim);padding:1px 0;border-bottom:1px solid #0d1117}
      .log-row:first-child{color:var(--accent2)}
      .us-bg  {height:5px;background:var(--border);border-radius:3px;overflow:hidden;margin-top:3px}
      .us-fill{height:100%;border-radius:3px;transition:width .3s}
      .btn-stop  {background:var(--danger)!important;color:#fff!important;font-weight:900!important;letter-spacing:2px!important}
      .btn-cancel{background:rgba(255,107,53,.15)!important;border:1px solid var(--warn)!important;color:var(--warn)!important}
      .btn-scan  {background:rgba(0,212,255,.12)!important;border:1px solid var(--accent)!important;color:var(--accent)!important}
      .btn-plant {background:rgba(0,230,118,.10)!important;border:1px solid var(--accent2)!important;color:var(--accent2)!important}
      .mode-active{outline:2px solid var(--warn)!important;animation:pulse 1s infinite}
      ::-webkit-scrollbar{width:4px}
      ::-webkit-scrollbar-track{background:var(--bg)}
      ::-webkit-scrollbar-thumb{background:var(--border);border-radius:2px}
    </style>
    """)

    # ── Header ───────────────────────────────────────────────
    with ui.header().classes('items-center justify-between px-4 py-2'):
        with ui.row().classes('items-center gap-3'):
            ui.html('<svg width="26" height="26" viewBox="0 0 26 26">'
                    '<polygon points="13,2 24,7.5 24,18.5 13,24 2,18.5 2,7.5" fill="none" stroke="#00d4ff" stroke-width="1.5"/>'
                    '<circle cx="13" cy="13" r="3.5" fill="#00d4ff"/></svg>')
            ui.label('AGV DIGITAL TWIN v3').style(
                'font-size:17px;font-weight:700;letter-spacing:3px;color:#00d4ff')
        nav_badge = ui.html('<span class="badge b-idle">IDLE</span>')

    # ── Layout ───────────────────────────────────────────────
    with ui.splitter(value=65).classes('w-full').style('height:calc(100vh - 52px)') as spl:

        # ═════════════ LEFT: map + camera ═════════════
        with spl.before:
            with ui.column().classes('w-full gap-2 p-2 overflow-y-auto'):

                # Position strip
                with ui.card().style('padding:8px 14px'):
                    with ui.row().classes('items-center gap-6'):
                        ui.label('POSITION').classes('ptitle').style('margin:0;padding:0')
                        pos_x   = ui.label('0.000').classes('coord')
                        ui.label('x').style('font-size:9px;color:var(--dim)')
                        pos_y   = ui.label('0.000').classes('coord')
                        ui.label('y').style('font-size:9px;color:var(--dim)')
                        pos_yaw = ui.label('0.0°').classes('coord')
                        ui.label('yaw').style('font-size:9px;color:var(--dim)')
                        ui.html('&nbsp;'*4)
                        op_badge = ui.html('<span class="badge b-idle">IDLE</span>')

                # Map
                with ui.card().style('padding:10px'):
                    with ui.row().classes('items-center justify-between'):
                        ui.label('LIVE MAP').classes('ptitle')
                        mode_hint = ui.label('').style(
                            'font-size:11px;color:var(--warn);font-family:var(--mono)')
                    plot = ui.plotly(_build_map_fig(
                        0, 0, 0, {}, [], 0, 0, False, plants, None, [], 0,
                    )).classes('w-full')
                    plot.on('plotly_click', lambda e: _on_map_click(e))

                # Camera
                with ui.card().style('padding:10px'):
                    ui.label('CAMERA FEED').classes('ptitle')
                    ui.html('<img src="/video_feed" style="width:640px;height:480px;'
                            'object-fit:contain;background:#050a0f;border-radius:6px;'
                            'max-width:100%;display:block">')

        # ═════════════ RIGHT: controls ═════════════
        with spl.after:
            with ui.column().classes('w-full gap-2 p-2 overflow-y-auto'):

                # E-STOP + Cancel
                with ui.row().classes('w-full gap-2'):
                    ui.button('⚠  E-STOP', on_click=lambda: _do_estop()) \
                        .classes('btn-stop flex-1').style('padding:12px;font-size:15px')
                    ui.button('✕  Cancel Nav', on_click=lambda: _do_cancel()) \
                        .classes('btn-cancel flex-1').style('padding:12px')

                # Nav coords
                with ui.card().style('padding:10px'):
                    ui.label('NAVIGATION').classes('ptitle')
                    with ui.row().classes('items-center gap-2 w-full'):
                        nav_x = ui.input(placeholder='X (m)').style(
                            'flex:1;background:#080e18;color:var(--accent2);'
                            'border:1px solid var(--border);border-radius:6px;'
                            'padding:6px 10px;font-family:var(--mono)')
                        nav_y = ui.input(placeholder='Y (m)').style(
                            'flex:1;background:#080e18;color:var(--accent2);'
                            'border:1px solid var(--border);border-radius:6px;'
                            'padding:6px 10px;font-family:var(--mono)')
                        ui.button('GO', on_click=lambda: _go_to_point(nav_x.value, nav_y.value)) \
                            .style('background:rgba(0,212,255,.18);border:1px solid var(--accent);'
                                   'color:var(--accent);font-weight:700;padding:8px 16px')

                # Manual drive
                with ui.card().style('padding:10px'):
                    ui.label('MANUAL DRIVE  (WASD)').classes('ptitle')
                    spd = ui.slider(min=0.1, max=1.0, step=0.05, value=0.3) \
                            .classes('w-full').props('label-always color=cyan')
                    with ui.column().classes('items-center gap-1'):
                        _btn_s = ('min-width:48px;height:48px;font-size:18px;'
                                  'background:rgba(0,212,255,.15);border:1px solid var(--accent);color:var(--accent)')
                        _btn_n = ('min-width:48px;height:48px;font-size:18px;'
                                  'background:rgba(0,212,255,.08);border:1px solid var(--border);color:var(--text)')
                        with ui.row().classes('justify-center'):
                            ui.button('▲', on_click=lambda: ctrl.send_vel(spd.value, 0)).style(_btn_s)
                        with ui.row().classes('justify-center gap-2'):
                            ui.button('◄', on_click=lambda: ctrl.send_vel(0, spd.value)).style(_btn_n)
                            ui.button('■', on_click=lambda: ctrl.stop()).style(
                                'min-width:48px;height:48px;font-size:18px;'
                                'background:rgba(255,34,68,.18);border:1px solid var(--danger);color:var(--danger)')
                            ui.button('►', on_click=lambda: ctrl.send_vel(0, -spd.value)).style(_btn_n)
                        with ui.row().classes('justify-center'):
                            ui.button('▼', on_click=lambda: ctrl.send_vel(-spd.value, 0)).style(_btn_s)

                # Ultrasonics
                with ui.card().style('padding:10px'):
                    ui.label('ULTRASONICS').classes('ptitle')
                    us_labels: dict = {}
                    us_bars:   dict = {}
                    us_names = {
                        'us1': 'US1 · Right',
                        'us2': 'US2 · Front-Right',
                        'us3': 'US3 · Front-Left',
                        'us4': 'US4 · Left',
                    }
                    for k, name in us_names.items():
                        with ui.column().classes('w-full').style('margin-bottom:8px'):
                            with ui.row().classes('w-full justify-between items-center'):
                                ui.label(name).style('font-size:11px;font-weight:600')
                                us_labels[k] = ui.html(
                                    '<span style="font-family:var(--mono);font-size:12px;color:var(--accent2)">-- m</span>')
                            us_bars[k] = ui.html(
                                '<div class="us-bg"><div class="us-fill" style="width:0%;background:var(--accent)"></div></div>')

                # Mission control
                with ui.card().style('padding:10px'):
                    ui.label('MISSION CONTROL').classes('ptitle')
                    with ui.row().classes('items-center gap-2').style('margin-bottom:6px'):
                        ui.label('Samples').style('font-size:11px;color:var(--dim)')
                        n_input = ui.number(value=SOIL_SCAN_N_DEFAULT, min=1, max=20,
                                            format='%.0f').style(
                            'width:64px;background:#080e18;color:var(--accent2);'
                            'border:1px solid var(--border);border-radius:6px')
                    with ui.row().classes('items-center gap-2 w-full').style('margin-bottom:8px'):
                        ui.label('Seed').style('font-size:11px;color:var(--dim)')
                        seed_sel = ui.select(
                            options=seed_options,
                            value=list(seed_options.keys())[0],
                        ).style('flex:1;background:#080e18;color:var(--accent2)')
                    with ui.row().classes('w-full gap-2'):
                        btn_scan  = ui.button('🔍  Soil Scan',
                                              on_click=lambda: _start_soil_scan(int(n_input.value))) \
                                        .classes('btn-scan flex-1')
                        btn_plant = ui.button('🌱  Plant Region',
                                              on_click=lambda: _start_plant_region()) \
                                        .classes('btn-plant flex-1')

                # Waypoints
                with ui.card().style('padding:10px'):
                    ui.label('WAYPOINTS').classes('ptitle')
                    wp_list = ui.column().classes('w-full')

                # Log
                with ui.card().style('padding:10px'):
                    ui.label('ACTIVITY LOG').classes('ptitle')
                    log_col = ui.column().classes('w-full').style('max-height:180px;overflow-y:auto')

                # Legend
                with ui.card().style('padding:10px'):
                    ui.label('MAP LEGEND').classes('ptitle')
                    legend = [('#808080', 'Not planted'), ('#FFD700', 'Detected')]
                    legend += [(c, f'Batch {i + 1}') for i, c in enumerate(BATCH_COLOURS[:5])]
                    with ui.grid(columns=2).classes('w-full gap-1'):
                        for col_hex, lbl in legend:
                            with ui.row().classes('items-center gap-2'):
                                ui.html(f'<div style="width:13px;height:13px;background:{col_hex};'
                                        f'border-radius:2px;border:1px solid rgba(255,255,255,.2)"></div>')
                                ui.label(lbl).style('font-size:11px;color:var(--dim)')

    # ════════════════════════════════════════════════════════
    #  Interaction handlers (closures over UI elements above)
    # ════════════════════════════════════════════════════════

    def _refresh_log():
        log_col.clear()
        with log_col:
            for entry in logs[:30]:
                ui.html(f'<div class="log-row">{entry}</div>')

    def _refresh_waypoints():
        wp_list.clear()
        wps = st.mission_waypoints
        idx = st.mission_wp_idx
        with wp_list:
            if not wps:
                ui.label('No active waypoints').style('font-size:11px;color:var(--dim)')
                return
            for i, (wx, wy) in enumerate(wps):
                done   = i < idx
                active = i == idx
                colour = ('var(--dim)' if done
                          else ('var(--accent2)' if active else 'var(--accent)'))
                icon   = '✓' if done else ('▶' if active else f'{i + 1}.')
                ui.label(f'{icon}  WP{i + 1}  ({wx:.2f}, {wy:.2f})').style(
                    f'font-family:var(--mono);font-size:11px;color:{colour}')

    def _on_map_click(e):
        try:
            pts = e.args['points']
            x = pts[0]['x']
            y = pts[0]['y']
        except (KeyError, IndexError, TypeError):
            return

        mode = click_mode['mode']
        if mode is None:
            nav_x.value = round(x, 2)
            nav_y.value = round(y, 2)
            _go_to_point(x, y)
            return

        # Two-corner area selection
        first = area_corner['pt']
        if first is None:
            area_corner['pt'] = (x, y)
            mode_hint.set_text(f'2nd corner…  start=({x:.2f}, {y:.2f})')
            _log(f'Area 1st corner ({x:.2f}, {y:.2f}) — click 2nd corner')
        else:
            x0, y0 = min(first[0], x), min(first[1], y)
            x1, y1 = max(first[0], x), max(first[1], y)
            area_corner['pt'] = None
            mode_hint.set_text('')

            if mode == 'soil_scan_area':
                pending_area['soil'] = (x0, y0, x1, y1)
                click_mode['mode'] = None
                btn_scan.classes(remove='mode-active')
                _log(f'Soil scan area locked  ({x0:.2f},{y0:.2f})→({x1:.2f},{y1:.2f})')
                _confirm_soil_scan()
            elif mode == 'plant_area':
                pending_area['plant'] = (x0, y0, x1, y1)
                click_mode['mode'] = None
                btn_plant.classes(remove='mode-active')
                _log(f'Plant region locked  ({x0:.2f},{y0:.2f})→({x1:.2f},{y1:.2f})')
                _confirm_plant_region()

    def _go_to_point(x, y):
        try:
            fx, fy = float(x), float(y)
        except (ValueError, TypeError):
            _log('Invalid coordinates')
            return
        ctrl.navigate_to(fx, fy)
        _log(f'Nav2 goal → ({fx:.2f}, {fy:.2f})')

    def _do_estop():
        ctrl.abort_mission()
        ctrl.stop()
        _log('⚠  E-STOP activated')

    def _do_cancel():
        ctrl.abort_mission()
        _log('Navigation / mission cancelled')

    def _start_soil_scan(n: int):
        click_mode['mode'] = 'soil_scan_area'
        area_corner['pt']  = None
        mode_hint.set_text(f'Click 1st corner for soil scan ({n} samples)…')
        btn_scan.classes('mode-active')
        _log(f'Soil scan — click 2 corners on map  ({n} samples)')

    def _confirm_soil_scan():
        area = pending_area.get('soil')
        if not area:
            return
        n = int(n_input.value)
        x0, y0, x1, y1 = area
        wps = [(random.uniform(x0, x1), random.uniform(y0, y1)) for _ in range(n)]
        _log(f'Generated {n} random sample waypoints')
        asyncio.ensure_future(ctrl.run_soil_scan(wps, SOIL_SCAN_DWELL, _log))

    def _start_plant_region():
        click_mode['mode'] = 'plant_area'
        area_corner['pt']  = None
        mode_hint.set_text('Click 1st corner for plant region…')
        btn_plant.classes('mode-active')
        _log('Plant region — click 2 corners on map')

    def _confirm_plant_region():
        area = pending_area.get('plant')
        if not area:
            return
        x0, y0, x1, y1 = area
        hs = PLANT_CELL_SIZE / 2
        seen: set = set()
        wps: list[tuple[float, float]] = []
        for p in plants:
            px_w = float(p['x'])
            py_w = float(p['y'])
            cx = math.floor(px_w / PLANT_CELL_SIZE) * PLANT_CELL_SIZE + hs
            cy = math.floor(py_w / PLANT_CELL_SIZE) * PLANT_CELL_SIZE + hs
            if x0 <= cx <= x1 and y0 <= cy <= y1:
                k = (round(cx, 3), round(cy, 3))
                if k not in seen:
                    seen.add(k)
                    wps.append((cx, cy))
        if not wps:
            _log('Plant region: no plant cells found in selected area')
            return
        _log(f'Plant region: {len(wps)} cell-centre waypoints queued')
        asyncio.ensure_future(ctrl.run_planting(wps, _log))

    # ════════════════════════════════════════════════════════
    #  Periodic UI refresh  (5 Hz)
    # ════════════════════════════════════════════════════════
    async def _tick():
        # Position
        pos_x.set_text(f'{st.x:.3f}')
        pos_y.set_text(f'{st.y:.3f}')
        pos_yaw.set_text(f'{math.degrees(st.yaw):.1f}°')

        # Badges
        op = st.operation
        ns = st.nav_status
        if op == 'soil_scan':
            op_badge.set_content('<span class="badge b-active">SCANNING</span>')
            nav_badge.set_content(f'<span class="badge b-active">SOIL SCAN</span>')
        elif op == 'planting':
            op_badge.set_content('<span class="badge b-warn">PLANTING</span>')
            nav_badge.set_content('<span class="badge b-warn">PLANTING</span>')
        elif st.nav_active:
            op_badge.set_content('<span class="badge b-active">NAVIGATING</span>')
            nav_badge.set_content(f'<span class="badge b-active">{ns[:26]}</span>')
        elif 'stop' in ns.lower() or 'cancel' in ns.lower():
            op_badge.set_content('<span class="badge b-danger">STOPPED</span>')
            nav_badge.set_content(f'<span class="badge b-danger">{ns[:26]}</span>')
        else:
            op_badge.set_content('<span class="badge b-idle">IDLE</span>')
            nav_badge.set_content('<span class="badge b-idle">IDLE</span>')

        # Ultrasonics
        for k in ('us1', 'us2', 'us3', 'us4'):
            d = st.us.get(k, float('inf'))
            finite = math.isfinite(d) and d < 10
            txt = f'{d:.2f} m' if finite else '-- m'
            if finite and d < 0.5:
                col = 'var(--danger)'
            elif finite and d < 1.5:
                col = 'var(--warn)'
            else:
                col = 'var(--accent2)'
            us_labels[k].set_content(
                f'<span style="font-family:var(--mono);font-size:12px;color:{col}">{txt}</span>')
            pct = (1.0 - min(d, US_MAX_RANGE) / US_MAX_RANGE) * 100 if finite else 0
            bc = 'var(--danger)' if pct > 80 else ('var(--warn)' if pct > 50 else 'var(--accent)')
            us_bars[k].set_content(
                f'<div class="us-bg"><div class="us-fill" '
                f'style="width:{pct:.0f}%;background:{bc}"></div></div>')

        # Waypoint list
        _refresh_waypoints()

        # Map redraw
        sel = pending_area.get('soil') or pending_area.get('plant')
        fig = _build_map_fig(
            st.x, st.y, st.yaw,
            st.us, st.path,
            st.goal_x, st.goal_y, st.has_goal,
            plants, sel,
            st.mission_waypoints, st.mission_wp_idx,
        )
        plot.update_figure(fig)

    ui.timer(0.20, _tick)

    # Keyboard WASD
    def _on_key(e):
        k = getattr(e, 'key', '')
        v = spd.value
        if   k == 'w':  ctrl.send_vel(v, 0)
        elif k == 's':  ctrl.send_vel(-v, 0)
        elif k == 'a':  ctrl.send_vel(0, v)
        elif k == 'd':  ctrl.send_vel(0, -v)
        elif k == ' ':  ctrl.stop()

    ui.keyboard(on_key=_on_key)
    _log('GUI ready — AgriBot v3')


# ═══════════════════════════════════════════════════════════════
#  Entry-point
# ═══════════════════════════════════════════════════════════════
def main():
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
    except ImportError:
        pass
    _ensure_singletons()
    ui.run(title='AgriBot v3', host='0.0.0.0', port=8080, reload=False, show=True)


if __name__ == '__main__':
    main()