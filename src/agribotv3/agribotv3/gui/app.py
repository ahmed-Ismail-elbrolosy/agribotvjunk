#!/usr/bin/env python3
"""AgriBot v3 dashboard (NiceGUI) with Nav2-compatible mission workflows."""
from __future__ import annotations

import asyncio
import math
import os
import time

import numpy as np
from nicegui import app, ui
from starlette.responses import StreamingResponse

from .config import (
    BATCH_COLOURS,
    NAV2_TOPICS,
    PLANT_CELL_SIZE,
    SOIL_SCAN_DWELL,
    SOIL_SCAN_N_DEFAULT,
    US_MAX_RANGE,
    load_plants_csv,
    load_seeds_csv,
)
from .controller import RosController
from .database import AgriBotDB
from .map_builder import build_map_figure, legend_rows
from .missions import plant_cell_centres_in_area, random_waypoints_in_area

ctrl: RosController | None = None
db: AgriBotDB | None = None


def _ensure_singletons() -> tuple[RosController, AgriBotDB]:
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
    return ctrl, db


@app.get('/video_feed')
async def video_feed():
    import cv2

    async def _stream():
        while True:
            try:
                if ctrl and ctrl.state.cam_data and ctrl.state.cam_w > 0:
                    state = ctrl.state
                    channels = 4 if 'a' in state.cam_encoding else 3
                    raw = state.cam_data
                    if raw and len(raw) == state.cam_h * state.cam_w * channels:
                        frame = np.frombuffer(raw, dtype=np.uint8).reshape(state.cam_h, state.cam_w, channels)
                        if channels == 4:
                            frame = frame[:, :, :3]
                        if state.cam_encoding.startswith('rgb'):
                            frame = frame[:, :, ::-1]
                        _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'
            except Exception:
                pass
            await asyncio.sleep(0.05)

    return StreamingResponse(_stream(), media_type='multipart/x-mixed-replace; boundary=frame')


class DashboardPage:
    def __init__(self, ros_ctrl: RosController, database: AgriBotDB):
        self.ctrl = ros_ctrl
        self.db = database
        self.state = ros_ctrl.state

        self.logs: list[str] = []
        self.click_mode: str | None = None
        self.first_corner: tuple[float, float] | None = None
        self.pending_area: dict[str, tuple[float, float, float, float]] = {}

        self.plants = load_plants_csv()
        self.seeds = load_seeds_csv()
        self.seed_options = (
            {seed['seed_id']: f"{seed['seed_id']} – {seed['name']}" for seed in self.seeds}
            if self.seeds
            else {'': 'No seeds found'}
        )

    def build(self) -> None:
        self._inject_styles()
        self._build_header()
        self._build_layout()
        ui.timer(0.2, self._tick)
        ui.keyboard(on_key=self._on_key)
        self._log('GUI ready — AgriBot v3 (structured Nav2-ready layout)')

    def _inject_styles(self) -> None:
        ui.add_head_html("""
        <style>
          :root { --bg:#0d1117; --panel:#111827; --border:#1e3a5f; --accent:#00d4ff;
            --accent2:#00e676; --warn:#ff6b35; --danger:#ff2244; --text:#c8d8e8; --dim:#4a6080;
            --mono:'Share Tech Mono',monospace; --sans:'Rajdhani',sans-serif; }
          body,.nicegui-content{background:var(--bg)!important;color:var(--text)!important} *{font-family:var(--sans)}
          .q-card{background:var(--panel)!important;border:1px solid var(--border)!important;border-radius:10px!important}
          .q-header{background:#080e18!important;border-bottom:1px solid var(--border)!important}
          .ptitle{font-size:10px;font-weight:700;letter-spacing:2px;text-transform:uppercase;color:var(--accent)}
          .badge{display:inline-block;padding:2px 10px;border-radius:20px;font-family:var(--mono);font-size:11px;font-weight:600}
          .b-idle{background:rgba(0,212,255,.10);color:var(--accent);border:1px solid var(--accent)}
          .b-active{background:rgba(0,230,118,.10);color:var(--accent2);border:1px solid var(--accent2)}
          .b-warn{background:rgba(255,107,53,.15);color:var(--warn);border:1px solid var(--warn)}
          .b-danger{background:rgba(255,34,68,.20);color:var(--danger);border:1px solid var(--danger)}
          .coord{font-family:var(--mono);font-size:14px;color:var(--accent2)}
          .log-row{font-family:var(--mono);font-size:11px;color:var(--dim);padding:1px 0;border-bottom:1px solid #0d1117}
          .mode-active{outline:2px solid var(--warn)!important}
        </style>
        """)

    def _build_header(self) -> None:
        with ui.header().classes('items-center justify-between px-4 py-2'):
            ui.label('AGV DIGITAL TWIN v3').style('font-size:17px;font-weight:700;letter-spacing:3px;color:#00d4ff')
            self.nav_badge = ui.html('<span class="badge b-idle">IDLE</span>')

    def _build_layout(self) -> None:
        with ui.splitter(value=65).classes('w-full').style('height:calc(100vh - 52px)') as splitter:
            with splitter.before:
                self._build_left_panel()
            with splitter.after:
                self._build_right_panel()

    def _build_left_panel(self) -> None:
        with ui.column().classes('w-full gap-2 p-2 overflow-y-auto'):
            with ui.card().style('padding:8px 14px'):
                with ui.row().classes('items-center gap-4'):
                    ui.label('POSITION').classes('ptitle')
                    self.pos_x = ui.label('0.000').classes('coord')
                    self.pos_y = ui.label('0.000').classes('coord')
                    self.pos_yaw = ui.label('0.0°').classes('coord')
                    self.op_badge = ui.html('<span class="badge b-idle">IDLE</span>')

            with ui.card().style('padding:10px'):
                with ui.row().classes('items-center justify-between'):
                    ui.label('LIVE MAP').classes('ptitle')
                    self.mode_hint = ui.label('').style('font-size:11px;color:var(--warn)')
                self.plot = ui.plotly(build_map_figure(0, 0, 0, {}, [], 0, 0, False, self.plants, None, [], 0, []))
                self.plot.classes('w-full')
                self.plot.on('plotly_click', self._on_map_click)

            with ui.card().style('padding:10px'):
                ui.label('CAMERA FEED').classes('ptitle')
                ui.html('<img src="/video_feed" style="width:640px;height:480px;object-fit:contain;background:#050a0f;max-width:100%">')

    def _build_right_panel(self) -> None:
        with ui.column().classes('w-full gap-2 p-2 overflow-y-auto'):
            with ui.row().classes('w-full gap-2'):
                ui.button('⚠ E-STOP', on_click=self._do_estop).classes('flex-1')
                ui.button('✕ Cancel Nav', on_click=self._do_cancel).classes('flex-1')

            with ui.card().style('padding:10px'):
                ui.label('NAVIGATION').classes('ptitle')
                ui.label(f"Nav2 topics/actions: {NAV2_TOPICS['odometry']} · {NAV2_TOPICS['goal_action']}").style('font-size:11px;color:var(--dim)')
                with ui.row().classes('items-center gap-2 w-full'):
                    self.nav_x = ui.input(placeholder='X (m)').style('flex:1')
                    self.nav_y = ui.input(placeholder='Y (m)').style('flex:1')
                    ui.button('GO', on_click=lambda: self._go_to_point(self.nav_x.value, self.nav_y.value))

            with ui.card().style('padding:10px'):
                ui.label('MANUAL DRIVE (WASD)').classes('ptitle')
                self.speed = ui.slider(min=0.1, max=1.0, step=0.05, value=0.3).classes('w-full')
                with ui.row().classes('gap-2'):
                    ui.button('▲', on_click=lambda: self.ctrl.send_vel(self.speed.value, 0))
                    ui.button('■', on_click=self.ctrl.stop)
                    ui.button('▼', on_click=lambda: self.ctrl.send_vel(-self.speed.value, 0))

            with ui.card().style('padding:10px'):
                ui.label('MISSION CONTROL').classes('ptitle')
                self.sample_count = ui.number(value=SOIL_SCAN_N_DEFAULT, min=1, max=20, format='%.0f').style('width:80px')
                self.seed_select = ui.select(options=self.seed_options, value=list(self.seed_options.keys())[0])
                with ui.row().classes('w-full gap-2'):
                    self.btn_scan = ui.button('🔍 Soil Scan', on_click=self._start_soil_scan).classes('flex-1')
                    self.btn_plant = ui.button('🌱 Plant Region', on_click=self._start_plant_region).classes('flex-1')

            with ui.card().style('padding:10px'):
                ui.label('WAYPOINTS').classes('ptitle')
                self.wp_list = ui.column().classes('w-full')

            with ui.card().style('padding:10px'):
                ui.label('ACTIVITY LOG').classes('ptitle')
                self.log_col = ui.column().classes('w-full').style('max-height:180px;overflow-y:auto')

            with ui.card().style('padding:10px'):
                ui.label('MAP LEGEND').classes('ptitle')
                with ui.grid(columns=2).classes('w-full gap-1'):
                    for col_hex, label in legend_rows():
                        with ui.row().classes('items-center gap-2'):
                            ui.html(f'<div style="width:13px;height:13px;background:{col_hex};border-radius:2px"></div>')
                            ui.label(label).style('font-size:11px;color:var(--dim)')

    def _log(self, message: str) -> None:
        timestamp = time.strftime('%H:%M:%S')
        self.logs.insert(0, f'[{timestamp}] {message}')
        self.logs = self.logs[:200]
        self._refresh_log()

    def _refresh_log(self) -> None:
        self.log_col.clear()
        with self.log_col:
            for entry in self.logs[:30]:
                ui.html(f'<div class="log-row">{entry}</div>')

    def _refresh_waypoints(self) -> None:
        self.wp_list.clear()
        with self.wp_list:
            if not self.state.mission_waypoints:
                ui.label('No active waypoints').style('font-size:11px;color:var(--dim)')
                return
            for idx, (wx, wy) in enumerate(self.state.mission_waypoints):
                marker = '✓' if idx < self.state.mission_wp_idx else ('▶' if idx == self.state.mission_wp_idx else f'{idx + 1}.')
                ui.label(f'{marker}  WP{idx + 1}  ({wx:.2f}, {wy:.2f})').style('font-size:11px')

    def _on_map_click(self, event) -> None:
        try:
            x = event.args['points'][0]['x']
            y = event.args['points'][0]['y']
        except (KeyError, IndexError, TypeError):
            return

        if self.click_mode is None:
            self.nav_x.value = round(x, 2)
            self.nav_y.value = round(y, 2)
            self._go_to_point(x, y)
            return

        if self.first_corner is None:
            self.first_corner = (x, y)
            self.mode_hint.set_text(f'2nd corner… ({x:.2f}, {y:.2f})')
            return

        x0, y0 = min(self.first_corner[0], x), min(self.first_corner[1], y)
        x1, y1 = max(self.first_corner[0], x), max(self.first_corner[1], y)
        self.first_corner = None
        self.mode_hint.set_text('')

        if self.click_mode == 'soil_scan':
            self.pending_area['soil'] = (x0, y0, x1, y1)
            self.click_mode = None
            self.btn_scan.classes(remove='mode-active')
            self._confirm_soil_scan()
        elif self.click_mode == 'plant_region':
            self.pending_area['plant'] = (x0, y0, x1, y1)
            self.click_mode = None
            self.btn_plant.classes(remove='mode-active')
            self._confirm_plant_region()

    def _go_to_point(self, x, y) -> None:
        try:
            fx, fy = float(x), float(y)
        except (TypeError, ValueError):
            self._log('Invalid coordinates')
            return
        self.ctrl.navigate_to(fx, fy)
        self._log(f'Nav2 goal → ({fx:.2f}, {fy:.2f})')

    def _do_estop(self) -> None:
        self.ctrl.abort_mission()
        self.ctrl.stop()
        self._log('E-STOP activated')

    def _do_cancel(self) -> None:
        self.ctrl.abort_mission()
        self._log('Navigation / mission cancelled')

    def _start_soil_scan(self) -> None:
        self.click_mode = 'soil_scan'
        self.first_corner = None
        self.btn_scan.classes(add='mode-active')
        self.mode_hint.set_text('Click 2 map corners for soil scan')

    def _confirm_soil_scan(self) -> None:
        area = self.pending_area.get('soil')
        if not area:
            return
        count = int(self.sample_count.value)
        waypoints = random_waypoints_in_area(area, count)
        self._log(f'Generated {len(waypoints)} random waypoints')
        asyncio.ensure_future(self.ctrl.run_soil_scan(waypoints, SOIL_SCAN_DWELL, self._log))

    def _start_plant_region(self) -> None:
        self.click_mode = 'plant_region'
        self.first_corner = None
        self.btn_plant.classes(add='mode-active')
        self.mode_hint.set_text('Click 2 map corners for plant region')

    def _confirm_plant_region(self) -> None:
        area = self.pending_area.get('plant')
        if not area:
            return
        waypoints = plant_cell_centres_in_area(self.plants, area, PLANT_CELL_SIZE)
        if not waypoints:
            self._log('No plant cells found in selected area')
            return
        self._log(f'Plant region queued with {len(waypoints)} targets')
        asyncio.ensure_future(self.ctrl.run_planting(waypoints, self._log))

    async def _tick(self) -> None:
        self.pos_x.set_text(f'{self.state.x:.3f}')
        self.pos_y.set_text(f'{self.state.y:.3f}')
        self.pos_yaw.set_text(f'{math.degrees(self.state.yaw):.1f}°')

        if self.state.operation == 'soil_scan':
            self.op_badge.set_content('<span class="badge b-active">SCANNING</span>')
        elif self.state.operation == 'planting':
            self.op_badge.set_content('<span class="badge b-warn">PLANTING</span>')
        elif self.state.nav_active:
            self.op_badge.set_content('<span class="badge b-active">NAVIGATING</span>')
        else:
            self.op_badge.set_content('<span class="badge b-idle">IDLE</span>')

        self.nav_badge.set_content(f'<span class="badge b-idle">{self.state.nav_status[:28]}</span>')
        self._refresh_waypoints()

        active_area = self.pending_area.get('soil') or self.pending_area.get('plant')
        figure = build_map_figure(
            self.state.x,
            self.state.y,
            self.state.yaw,
            self.state.us,
            self.state.path,
            self.state.goal_x,
            self.state.goal_y,
            self.state.has_goal,
            self.plants,
            active_area,
            self.state.mission_waypoints,
            self.state.mission_wp_idx,
            self.db.get_all_cells(),
        )
        self.plot.update_figure(figure)

    def _on_key(self, event) -> None:
        key = getattr(event, 'key', '')
        speed = self.speed.value
        if key == 'w':
            self.ctrl.send_vel(speed, 0)
        elif key == 's':
            self.ctrl.send_vel(-speed, 0)
        elif key == 'a':
            self.ctrl.send_vel(0, speed)
        elif key == 'd':
            self.ctrl.send_vel(0, -speed)
        elif key == ' ':
            self.ctrl.stop()


@ui.page('/')
def index() -> None:
    ros_ctrl, database = _ensure_singletons()
    DashboardPage(ros_ctrl, database).build()


def main() -> None:
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    _ensure_singletons()
    ui.run(title='AgriBot v3', host='0.0.0.0', port=8080, reload=False, show=True)


if __name__ == '__main__':
    main()
