"""CSV-based persistence for grid cells, seeds & soil scans.

Thread-safe, zero external dependencies.  Reused from agribotv2 with the
data directory resolved relative to the installed *share* folder so that
``colcon build --symlink-install`` works correctly.
"""
import csv
import os
from datetime import datetime
from threading import Lock


class AgriBotDB:
    def __init__(self, data_dir: str | None = None):
        if data_dir is None:
            # Default: <pkg_share>/data  (set by the launch / app startup)
            data_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..', '..', '..', 'share', 'agribotv3', 'data',
            )
        self.data_dir = os.path.abspath(data_dir)
        os.makedirs(self.data_dir, exist_ok=True)

        self.cells_file = os.path.join(self.data_dir, 'grid_cells.csv')
        self.seeds_file = os.path.join(self.data_dir, 'seeds.csv')
        self.scans_file = os.path.join(self.data_dir, 'soil_scans.csv')

        self._lock = Lock()
        self._init_files()

    # ── bootstrap ────────────────────────────────────────────
    def _init_files(self):
        if not os.path.exists(self.cells_file):
            with open(self.cells_file, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['grid_x', 'grid_y', 'state', 'batch_id', 'timestamp', 'data', 'seed_type'])

        if not os.path.exists(self.seeds_file):
            with open(self.seeds_file, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['id', 'seed_name', 'planting_depth_cm',
                            'spacing_cm', 'germination_days', 'description'])
                w.writerows([
                    (1, 'Wheat',     3.0, 15.0, 7,  'Common cereal grain'),
                    (2, 'Corn',      5.0, 30.0, 10, 'Maize crop'),
                    (3, 'Soybean',   4.0,  7.5, 7,  'Legume crop'),
                    (4, 'Tomato',    1.5, 60.0, 7,  'Vegetable fruit'),
                    (5, 'Carrot',    1.0,  5.0, 14, 'Root vegetable'),
                    (6, 'Potato',   10.0, 30.0, 14, 'Tuber crop'),
                    (7, 'Lettuce',   0.5, 25.0, 7,  'Leafy green'),
                    (8, 'Sunflower', 2.5, 45.0, 10, 'Oilseed crop'),
                ])

        if not os.path.exists(self.scans_file):
            with open(self.scans_file, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'id', 'grid_x', 'grid_y', 'world_x', 'world_y', 'timestamp',
                    'nitrogen_ppm', 'phosphorus_ppm', 'potassium_ppm', 'ph_level',
                    'moisture_percent', 'temperature_celsius', 'ec_mS_cm', 'notes'])

    # ── Grid cells ───────────────────────────────────────────
    def _read_cells(self):
        rows = []
        if os.path.exists(self.cells_file):
            with open(self.cells_file, newline='') as f:
                r = csv.reader(f); next(r, None); rows = list(r)
        return rows

    def _write_cells(self, rows):
        with open(self.cells_file, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['grid_x', 'grid_y', 'state', 'batch_id',
                         'timestamp', 'data', 'seed_type'])
            w.writerows(rows)

    def update_cell(self, col, row, state, batch_id='MANUAL', data='', seed_type=None):
        with self._lock:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            cells = self._read_cells()
            found = False
            for i, c in enumerate(cells):
                if int(c[0]) == col and int(c[1]) == row:
                    if len(c) > 5 and str(c[5]).strip():
                        return
                    cells[i] = [col, row, state, batch_id, ts, data, seed_type or '']
                    found = True
                    break
            if not found:
                cells.append([col, row, state, batch_id, ts, data, seed_type or ''])
            self._write_cells(cells)

    def get_all_cells(self):
        with self._lock:
            out = []
            for c in self._read_cells():
                try:
                    out.append((int(c[0]), int(c[1]), c[2], c[3], c[4],
                                c[5] if len(c) > 5 else '',
                                c[6] if len(c) > 6 else ''))
                except (ValueError, IndexError):
                    pass
            return out

    def get_cell_status(self, row, col):
        for c in self.get_all_cells():
            if c[0] == col and c[1] == row:
                return c[2]
        return None

    def clear_cells(self):
        with self._lock:
            self._write_cells([])

    # ── Seeds ────────────────────────────────────────────────
    def _read_seeds(self):
        rows = []
        if os.path.exists(self.seeds_file):
            with open(self.seeds_file, newline='') as f:
                r = csv.reader(f); next(r, None); rows = list(r)
        return rows

    def _write_seeds(self, rows):
        with open(self.seeds_file, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['id', 'seed_name', 'planting_depth_cm',
                         'spacing_cm', 'germination_days', 'description'])
            w.writerows(rows)

    def add_seed(self, seed_name, depth, spacing=None, germ_days=None, desc=None):
        with self._lock:
            seeds = self._read_seeds()
            mx = 0
            for i, s in enumerate(seeds):
                try:
                    mx = max(mx, int(s[0]))
                    if s[1] == seed_name:
                        seeds[i] = [s[0], seed_name, depth,
                                    spacing or '', germ_days or '', desc or '']
                        self._write_seeds(seeds)
                        return
                except (ValueError, IndexError):
                    pass
            seeds.append([mx + 1, seed_name, depth,
                          spacing or '', germ_days or '', desc or ''])
            self._write_seeds(seeds)

    def get_all_seeds(self):
        with self._lock:
            out = []
            for s in self._read_seeds():
                try:
                    out.append((
                        int(s[0]), s[1],
                        float(s[2]) if s[2] else 0.0,
                        float(s[3]) if s[3] else None,
                        int(s[4])   if s[4] else None,
                        s[5] if len(s) > 5 else '',
                    ))
                except (ValueError, IndexError):
                    pass
            return out

    def get_seed_depth(self, name):
        for s in self.get_all_seeds():
            if s[1] == name:
                return s[2]
        return None

    def delete_seed(self, name):
        with self._lock:
            self._write_seeds([s for s in self._read_seeds() if s[1] != name])

    # ── Soil scans ───────────────────────────────────────────
    def _read_scans(self):
        rows = []
        if os.path.exists(self.scans_file):
            with open(self.scans_file, newline='') as f:
                r = csv.reader(f); next(r, None); rows = list(r)
        return rows

    def _write_scans(self, rows):
        with open(self.scans_file, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['id', 'grid_x', 'grid_y', 'world_x', 'world_y', 'timestamp',
                         'nitrogen_ppm', 'phosphorus_ppm', 'potassium_ppm', 'ph_level',
                         'moisture_percent', 'temperature_celsius', 'ec_mS_cm', 'notes'])
            w.writerows(rows)

    def add_soil_scan(self, gx, gy, wx, wy,
                      n=None, p=None, k=None, ph=None,
                      moist=None, temp=None, ec=None, notes=None):
        with self._lock:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            scans = self._read_scans()
            mx = max((int(s[0]) for s in scans if s[0].isdigit()), default=0)
            scans.append([mx + 1, gx, gy, wx, wy, ts,
                          n or '', p or '', k or '', ph or '',
                          moist or '', temp or '', ec or '', notes or ''])
            self._write_scans(scans)
            return mx + 1

    def get_all_soil_scans(self):
        with self._lock:
            out = []
            for s in self._read_scans():
                try:
                    out.append((
                        int(s[0]), int(s[1]), int(s[2]),
                        float(s[3]) if s[3] else 0.0,
                        float(s[4]) if s[4] else 0.0,
                        s[5],
                        *(float(s[i]) if s[i] else None for i in range(6, 13)),
                        s[13] if len(s) > 13 else '',
                    ))
                except (ValueError, IndexError):
                    pass
            out.sort(key=lambda x: x[5], reverse=True)
            return out

    def get_soil_scan_for_cell(self, gx, gy):
        for s in self.get_all_soil_scans():
            if s[1] == gx and s[2] == gy:
                return s
        return None

    def close(self):
        pass
