import os
import math
import random
from PyQt5.QtWidgets import QWidget, QSizePolicy
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QRect, QPoint, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor, QPixmap, QPainterPath, QPolygonF

import config

class MapWidget(QWidget):
    # Signal to update cursor label in MainWindow
    mouse_moved_signal = pyqtSignal(float, float) 

    def __init__(self, db, controller):
        super().__init__()
        self.db = db
        self.ctrl = controller
        
        # UI Settings
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        
        # Internal State
        self.rx = 0.0
        self.ry = 0.0
        self.ryaw = 0.0
        self.us_data = {}
        self.current_goal = None
        self.future_path = []
        
        # --- NEW: Pure Pursuit Visualization State ---
        self.pp_target = None  # Tuple (x, y)
        self.pp_robot_pos = None # Tuple (x, y) snapshot
        # ---------------------------------------------
        
        # Interaction State
        self.mode = "VIEW" 
        self.selection_start = None
        self.selection_end = None
        
        # Visuals
        if os.path.exists(config.BG_PATH):
            self.bg_pixmap = QPixmap(config.BG_PATH)
        else:
            self.bg_pixmap = None

        # Scaling Vars
        self.scale = 1.0
        self.off_x = 0
        self.off_y = 0
        
        # Blink Animation
        self.blink_pos = None
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.stop_blink)

    # ==========================================
    # SLOTS
    # ==========================================
    
    # --- NEW: Receive Pure Pursuit Data ---
    def update_pursuit(self, tx, ty, rx, ry):
        """Called by RosController to show where the robot is aiming."""
        self.pp_target = (tx, ty)
        self.pp_robot_pos = (rx, ry)
        self.update()
    # --------------------------------------

    def start_blink(self, x, y): 
        self.blink_pos = (x, y)
        self.blink_timer.start(1000)
        self.update()
    
    def stop_blink(self): 
        self.blink_pos = None
        self.blink_timer.stop()
        self.update()
        
    def update_sensors(self, data):
        self.us_data = data
        self.update()

    def update_path(self, goal, path):
        self.current_goal = goal
        self.future_path = path
        # If path is cleared, clear debug viz too
        if not path:
            self.pp_target = None
        self.update()

    def update_robot(self, x, y, yaw):
        self.rx = x
        self.ry = y
        self.ryaw = yaw
        self.update()

    # ==========================================
    # HELPERS
    # ==========================================
    def _batch_to_color(self, batch_id: str) -> QColor:
        if not batch_id or batch_id == 'MANUAL':
            return QColor(144, 238, 144, 120)
        batch_hash = hash(batch_id)
        hue = 70 + (abs(batch_hash) % 110)
        saturation = 120 + (abs(batch_hash >> 8) % 80)
        value = 180 + (abs(batch_hash >> 16) % 60)
        color = QColor()
        color.setHsv(hue, saturation, value, 130)
        return color
    
    _batch_color_cache = {}
    
    def get_batch_color(self, batch_id: str) -> QColor:
        if batch_id not in self._batch_color_cache:
            self._batch_color_cache[batch_id] = self._batch_to_color(batch_id)
        return self._batch_color_cache[batch_id]

    # ==========================================
    # COORDINATE TRANSFORMS
    # ==========================================
    def update_transform_params(self):
        w_px = self.width()
        h_px = self.height()
        world_w = config.WORLD_MAX_X - config.WORLD_MIN_X 
        world_h = config.WORLD_MAX_Y - config.WORLD_MIN_Y
        
        scale_x = w_px / world_w
        scale_y = h_px / world_h
        self.scale = min(scale_x, scale_y)
        
        draw_w = world_w * self.scale
        draw_h = world_h * self.scale
        self.off_x = (w_px - draw_w) / 2
        self.off_y = (h_px - draw_h) / 2

    def world_to_screen(self, wx, wy):
        if self.scale == 0: return 0,0 
        sx = self.off_x + ((wx - config.WORLD_MIN_X) * self.scale)
        sy = self.off_y + ((config.WORLD_MAX_Y - wy) * self.scale)
        return int(sx), int(sy)

    def screen_to_world(self, sx, sy):
        if self.scale == 0: return 0,0
        wx = config.WORLD_MIN_X + ((sx - self.off_x) / self.scale)
        wy = config.WORLD_MAX_Y - ((sy - self.off_y) / self.scale)
        return wx, wy

    # ==========================================
    # PAINT EVENT
    # ==========================================
    def paintEvent(self, event):
        self.update_transform_params()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # 1. Background
        map_rect = QRect(
            int(self.off_x), 
            int(self.off_y), 
            int((config.WORLD_MAX_X - config.WORLD_MIN_X) * self.scale), 
            int((config.WORLD_MAX_Y - config.WORLD_MIN_Y) * self.scale)
        )
        
        if self.bg_pixmap:
            bg_w = self.bg_pixmap.width()
            bg_h = self.bg_pixmap.height()
            target_w = map_rect.width()
            target_h = map_rect.height()
            scale_to_cover = max(target_w / bg_w, target_h / bg_h)
            scaled_w = int(bg_w * scale_to_cover)
            scaled_h = int(bg_h * scale_to_cover)
            crop_x = (scaled_w - target_w) // 2
            crop_y = (scaled_h - target_h) // 2
            scaled_pixmap = self.bg_pixmap.scaled(scaled_w, scaled_h, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
            cropped_pixmap = scaled_pixmap.copy(crop_x, crop_y, target_w, target_h)
            p.drawPixmap(map_rect.topLeft(), cropped_pixmap)
        else:
            p.fillRect(map_rect, QColor(238, 214, 175))
        
        p.setPen(QPen(Qt.black, 3)); p.setBrush(Qt.NoBrush); p.drawRect(map_rect)

        # 2. Grid & Cells
        self.draw_grid_lines(p)
        self.draw_cells(p)

        # 3. Path
        self.draw_path(p)

        # --- NEW: 3.5 Pure Pursuit Debug Visualization ---
        if self.pp_target and self.pp_robot_pos:
            tx, ty = self.world_to_screen(*self.pp_target)
            rx, ry = self.world_to_screen(*self.pp_robot_pos)
            
            # Draw Line (Robot -> Target)
            pen = QPen(Qt.red, 2, Qt.DashLine)
            p.setPen(pen)
            p.drawLine(rx, ry, tx, ty)
            
            # Draw Target Dot
            p.setBrush(Qt.red)
            p.setPen(Qt.black)
            p.drawEllipse(QPoint(tx, ty), 6, 6)
        # -------------------------------------------------

        # 4. Selection
        if self.selection_start and self.selection_end:
            rect = QRect(self.selection_start, self.selection_end).normalized()
            p.setPen(QPen(Qt.blue, 2, Qt.DashLine)); p.setBrush(QColor(0, 0, 255, 50)); p.drawRect(rect)

        # 5. Blink
        if self.blink_pos:
             bx, by = self.world_to_screen(self.blink_pos[0], self.blink_pos[1])
             p.setPen(QPen(Qt.red, 3)); p.setBrush(QColor(255, 0, 0, 100)); p.drawEllipse(QPoint(bx, by), 15, 15)

        # 6. Robot
        self.draw_robot_and_sensors(p)

    def draw_cells(self, p):
        # Fetch data
        cells = self.db.get_all_cells()
        known_map = {(c[0], c[1]): c for c in cells}
        
        for r in range(config.GRID_CONFIG['rows']):
            for c in range(config.GRID_CONFIG['cols']):
                # 1. Calculate Cell Coordinates
                wx_min = config.GRID_CONFIG['min_x'] + (c * config.CELL_W)
                wx_max = wx_min + config.CELL_W
                wy_max = config.GRID_CONFIG['max_y'] - (r * config.CELL_H)
                wy_min = wy_max - config.CELL_H
                
                s_tl_x, s_tl_y = self.world_to_screen(wx_min, wy_max)
                s_br_x, s_br_y = self.world_to_screen(wx_max, wy_min)
                rect = QRect(QPoint(s_tl_x, s_tl_y), QPoint(s_br_x, s_br_y)).normalized()
                
                # 2. Determine State
                status = None
                batch_id = ''
                
                if (c, r) in known_map:
                    cell_data = known_map[(c, r)]
                    # Safe access to status and batch_id
                    if len(cell_data) > 2: status = str(cell_data[2]).strip()
                    if len(cell_data) > 3: batch_id = cell_data[3]

                # 3. Draw Background (Transparent/Subtle)
                bg_color = QColor(255, 255, 224, 20) # Default transparent
                
                if status == 'plant_checked' or status == 'checked': 
                    bg_color = QColor(255, 215, 0, 50) # Gold transparent
                elif status == 'soil_analyzed':
                    bg_color = QColor(0, 0, 255, 30)   # Blue transparent
                
                p.fillRect(rect, bg_color)
                
                # Draw faint grid lines
                p.setPen(QPen(QColor(0, 0, 0, 40), 1))
                p.drawRect(rect)

                # 4. Draw OBJECTS (Plants/Detections)
                # !!! EVERYTHING BELOW MUST BE INSIDE THE LOOP !!!
                if status == 'planted' or status == 'detected':
                    
                    # Calculate center of the cell
                    plant_wx = wx_min + config.CELL_W / 2
                    plant_wy = wy_min + config.CELL_H / 2
                    
                    # Radius: 0.2m as requested
                    inner_half = 0.20 
                    
                    # Convert object bounds to screen coordinates
                    inner_tl_x, inner_tl_y = self.world_to_screen(plant_wx - inner_half, plant_wy + inner_half)
                    inner_br_x, inner_br_y = self.world_to_screen(plant_wx + inner_half, plant_wy - inner_half)
                    inner_rect = QRect(QPoint(inner_tl_x, inner_tl_y), QPoint(inner_br_x, inner_br_y)).normalized()
                    
                    # SAFETY: Ensure minimum 5px size so it's visible even if zoomed out
                    if inner_rect.width() < 5:
                        cx = (s_tl_x + s_br_x) // 2
                        cy = (s_tl_y + s_br_y) // 2
                        inner_rect = QRect(cx - 3, cy - 3, 6, 6)

                    if status == 'planted':
                        # Solid Green Circle
                        plant_col = self.get_batch_color(batch_id) # Use getter
                        plant_col = QColor(plant_col)
                        plant_col.setAlpha(255) # Solid
                        
                        p.setBrush(plant_col)
                        p.setPen(QPen(Qt.black, 1))
                        p.drawEllipse(inner_rect)
                        
                    elif status == 'detected':
                        # Cyan Box
                        p.setBrush(QColor(0, 255, 255, 150)) 
                        p.setPen(QPen(Qt.black, 1))
                        p.drawRect(inner_rect)

    def draw_path(self, p):
        pts = [QPoint(*self.world_to_screen(self.rx, self.ry))]
        for pt in self.future_path: pts.append(QPoint(*self.world_to_screen(pt['x'], pt['y'])))
        if len(pts) > 1:
            path_obj = QPainterPath(); path_obj.moveTo(pts[0])
            for pt in pts[1:]: path_obj.lineTo(pt)
            pen = QPen(QColor(50, 255, 50)); pen.setWidth(3); p.setPen(pen); p.setBrush(Qt.NoBrush); p.drawPath(path_obj)
            p.setBrush(QColor(255, 255, 0)); p.setPen(Qt.black)
            for pt in pts[1:]: p.drawEllipse(pt, 4, 4)

    def draw_robot_and_sensors(self, p):
        sx, sy = self.world_to_screen(self.rx, self.ry)
        p.save(); p.translate(sx, sy); p.rotate(-math.degrees(self.ryaw)) 
        w = int(config.ROBOT_WIDTH * self.scale); l = int(config.ROBOT_LENGTH * self.scale)
        p.setBrush(Qt.blue); p.setPen(Qt.black); p.drawRect(-l//2, -w//2, l, w)
        arrow_len = l * 0.6; arrow_w = w * 0.6
        arrow = QPolygonF([QPointF(arrow_len/2 + l*0.1, 0), QPointF(-arrow_len/2, -arrow_w/2), QPointF(-arrow_len/2 + l*0.1, 0), QPointF(-arrow_len/2, arrow_w/2)])
        p.setBrush(QColor(255, 255, 0)); p.setPen(QPen(Qt.black, 1)); p.drawPolygon(arrow)
        
        # Front reference point
        front_offset_px = int(0.8 * self.scale)
        p.setBrush(QColor(255, 0, 255))
        p.setPen(QPen(Qt.black, 2))
        p.drawEllipse(QPoint(front_offset_px, 0), 8, 8)
        
        for name, conf in config.SENSOR_OFFSETS.items():
            r = self.us_data.get(name, 4.0)
            if r < 1.0: col = QColor(255, 0, 0, 150)
            elif r < 2.0: col = QColor(255, 165, 0, 100)
            else: col = QColor(0, 255, 255, 40)
            px = int(conf['x'] * self.scale); py = int(conf['y'] * self.scale) 
            p.save(); p.translate(px, -py); p.rotate(-math.degrees(conf['yaw']))
            r_px = int(r * self.scale); span = int(math.degrees(config.US_FOV) * 16)
            p.setBrush(col); p.setPen(Qt.NoPen); p.drawPie(-r_px, -r_px, r_px*2, r_px*2, -span//2, span); p.restore()
        p.restore()

    def draw_grid_lines(self, p):
        p.setPen(QPen(QColor(50, 50, 50, 100), 1)) 
        for y in range(int(config.WORLD_MIN_Y), int(config.WORLD_MAX_Y) + 1):
            sx1, sy1 = self.world_to_screen(config.WORLD_MIN_X, y); sx2, sy2 = self.world_to_screen(config.WORLD_MAX_X, y); p.drawLine(sx1, sy1, sx2, sy2)
        for x in range(int(config.WORLD_MIN_X), int(config.WORLD_MAX_X) + 1):
            sx1, sy1 = self.world_to_screen(x, config.WORLD_MIN_Y); sx2, sy2 = self.world_to_screen(x, config.WORLD_MAX_Y); p.drawLine(sx1, sy1, sx2, sy2)

    def mousePressEvent(self, event):
        wx, wy = self.screen_to_world(event.x(), event.y())
        if self.mode == "GOTO" or self.mode == "VIEW":
            if event.button() == Qt.LeftButton:
                self.ctrl.set_path([{'x': wx, 'y': wy, 'type': 'move'}], "MOVE")
                self.mode = "VIEW"; self.setCursor(Qt.ArrowCursor); self.update()
        elif self.mode != "VIEW": self.selection_start = event.pos(); self.selection_end = event.pos()

    def mouseReleaseEvent(self, event):
        if self.selection_start: self.process_selection(); self.selection_start = None; self.selection_end = None; self.update()
    
    def mouseMoveEvent(self, event):
        wx, wy = self.screen_to_world(event.x(), event.y())
        self.mouse_moved_signal.emit(wx, wy)
        if self.selection_start: self.selection_end = event.pos(); self.update()

    def process_selection(self):
        wx1, wy1 = self.screen_to_world(self.selection_start.x(), self.selection_start.y())
        wx2, wy2 = self.screen_to_world(self.selection_end.x(), self.selection_end.y())
        min_wx, max_wx = min(wx1, wx2), max(wx1, wx2); min_wy, max_wy = min(wy1, wy2), max(wy1, wy2)
        
        path = []; job = "MOVE"
        
        if self.mode == "PLANT":
            job = "PLANTING"
            cells = []
            for r in range(config.GRID_CONFIG['rows']):
                for c in range(config.GRID_CONFIG['cols']):
                    cx = config.GRID_CONFIG['min_x'] + (c + 0.5) * config.CELL_W
                    cy = config.GRID_CONFIG['min_y'] + (r + 0.5) * config.CELL_H
                    if min_wx <= cx <= max_wx and min_wy <= cy <= max_wy:
                        state = None
                        if hasattr(self.ctrl, 'db_ref') and self.ctrl.db_ref:
                            state = self.ctrl.db_ref.get_cell_status(r, c)
                        if state in ['planted', 'detected', 'obstacle']:
                            continue
                        cells.append({'r': r, 'c': c, 'x': cx, 'y': cy})
            if cells: path = self._generate_zigzag_path(cells)

        elif self.mode == "SOIL_SCAN":
            job = "SOIL"
            count = 3 
            try: count = self.parent().parent().parent().n_points_spin.value()
            except: pass
            for _ in range(count):
                 path.append({'x': random.uniform(min_wx, max_wx), 'y': random.uniform(min_wy, max_wy), 'type': 'soil_scan'})

        if path: self.ctrl.set_path(path, job)
        self.mode = "VIEW"; self.setCursor(Qt.ArrowCursor)
    
    def _generate_zigzag_path(self, cells):
        if not cells: return []
        robot_x, robot_y = self.ctrl.rx, self.ctrl.ry
        rows = {}
        for cell in cells:
            r = cell['r']
            if r not in rows: rows[r] = []
            rows[r].append(cell)
        sorted_row_indices = sorted(rows.keys())
        for r in rows: rows[r].sort(key=lambda c: c['c'])
        
        min_row, max_row = min(sorted_row_indices), max(sorted_row_indices)
        first_row_cells = rows[min_row]; last_row_cells = rows[max_row]
        
        corners = [
            (first_row_cells[0]['x'], first_row_cells[0]['y'], 'top-left', False),
            (first_row_cells[-1]['x'], first_row_cells[-1]['y'], 'top-right', True),
            (last_row_cells[0]['x'], last_row_cells[0]['y'], 'bottom-left', False),
            (last_row_cells[-1]['x'], last_row_cells[-1]['y'], 'bottom-right', True)
        ]
        
        closest = min(corners, key=lambda c: math.hypot(c[0] - robot_x, c[1] - robot_y))
        start_from_top = 'top' in closest[2]
        start_going_left = closest[3]
        
        path = []
        row_order = sorted_row_indices if start_from_top else sorted_row_indices[::-1]
        going_left = start_going_left
        
        for r in row_order:
            row_cells = rows[r]
            if going_left: row_cells = row_cells[::-1]
            for cell in row_cells:
                path.append({'x': cell['x'], 'y': cell['y'], 'type': 'plant', 'grid': (cell['c'], cell['r'])})
            going_left = not going_left
        return path