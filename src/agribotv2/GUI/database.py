import csv
import os
from datetime import datetime
from threading import Lock

class AgriBotDB:
    def __init__(self, data_dir="agribot_data"):
        # Create data directory if it doesn't exist
        self._script_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(self._script_dir, data_dir)
        os.makedirs(self.data_dir, exist_ok=True)
        
        # CSV file paths
        self.cells_file = os.path.join(self.data_dir, "grid_cells.csv")
        self.seeds_file = os.path.join(self.data_dir, "seeds.csv")
        self.scans_file = os.path.join(self.data_dir, "soil_scans.csv")
        
        # Thread safety
        self._lock = Lock()
        
        # Initialize CSV files
        self._init_files()

    def _init_files(self):
        """Create CSV files with headers if they don't exist"""
        # Grid cells
        if not os.path.exists(self.cells_file):
            with open(self.cells_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['grid_x', 'grid_y', 'state', 'batch_id', 'timestamp', 'data', 'seed_type'])
        
        # Seeds catalog
        if not os.path.exists(self.seeds_file):
            with open(self.seeds_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['id', 'seed_name', 'planting_depth_cm', 'spacing_cm', 'germination_days', 'description'])
                # Default seeds
                default_seeds = [
                    (1, 'Wheat', 3.0, 15.0, 7, 'Common cereal grain'),
                    (2, 'Corn', 5.0, 30.0, 10, 'Maize crop'),
                    (3, 'Soybean', 4.0, 7.5, 7, 'Legume crop'),
                    (4, 'Tomato', 1.5, 60.0, 7, 'Vegetable fruit'),
                    (5, 'Carrot', 1.0, 5.0, 14, 'Root vegetable'),
                    (6, 'Potato', 10.0, 30.0, 14, 'Tuber crop'),
                    (7, 'Lettuce', 0.5, 25.0, 7, 'Leafy green'),
                    (8, 'Sunflower', 2.5, 45.0, 10, 'Oilseed crop'),
                ]
                writer.writerows(default_seeds)
        
        # Soil scans - 7 readings: N, P, K, pH, Moisture, Temperature, EC
        if not os.path.exists(self.scans_file):
            with open(self.scans_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['id', 'grid_x', 'grid_y', 'world_x', 'world_y', 'timestamp',
                                'nitrogen_ppm', 'phosphorus_ppm', 'potassium_ppm', 'ph_level',
                                'moisture_percent', 'temperature_celsius', 'ec_mS_cm', 'notes'])

    # ============ Grid Cells Methods ============
    def update_cell(self, col, row, state, batch_id="MANUAL", data="", seed_type=None):
        """Update or insert a grid cell"""
        with self._lock:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cells = self._read_cells()
            
            # Find and update existing cell or add new
            found = False
            for i, cell in enumerate(cells):
                if int(cell[0]) == col and int(cell[1]) == row:
                    # If data already present, keep existing entry (requested behavior)
                    if len(cell) > 5 and str(cell[5]).strip() != '':
                        return
                    cells[i] = [col, row, state, batch_id, timestamp, data, seed_type or '']
                    found = True
                    break
            
            if not found:
                cells.append([col, row, state, batch_id, timestamp, data, seed_type or ''])
            
            self._write_cells(cells)

    def _read_cells(self):
        """Read all cells from CSV"""
        cells = []
        if os.path.exists(self.cells_file):
            with open(self.cells_file, 'r', newline='') as f:
                reader = csv.reader(f)
                next(reader, None)  # Skip header
                cells = list(reader)
        return cells

    def _write_cells(self, cells):
        """Write all cells to CSV"""
        with open(self.cells_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['grid_x', 'grid_y', 'state', 'batch_id', 'timestamp', 'data', 'seed_type'])
            writer.writerows(cells)

    def get_all_cells(self):
        """Returns list of tuples: (grid_x, grid_y, state, batch_id, timestamp, data, seed_type)"""
        with self._lock:
            cells = self._read_cells()
            # Convert to tuples with proper types
            result = []
            for cell in cells:
                try:
                    result.append((
                        int(cell[0]),      # grid_x
                        int(cell[1]),      # grid_y
                        cell[2],           # state
                        cell[3],           # batch_id
                        cell[4],           # timestamp
                        cell[5] if len(cell) > 5 else '',  # data
                        cell[6] if len(cell) > 6 else ''   # seed_type
                    ))
                except (ValueError, IndexError):
                    continue
            return result

    def get_cell_status(self, row, col):
        """Get the status of a specific cell by row, col. Returns None if not found."""
        cells = self.get_all_cells()
        for cell in cells:
            if cell[0] == col and cell[1] == row:
                return cell[2]  # state
        return None

    def clear_cells(self):
        """Clear all grid cells"""
        with self._lock:
            self._write_cells([])

    # ============ Seeds Methods ============
    def _read_seeds(self):
        """Read all seeds from CSV"""
        seeds = []
        if os.path.exists(self.seeds_file):
            with open(self.seeds_file, 'r', newline='') as f:
                reader = csv.reader(f)
                next(reader, None)  # Skip header
                seeds = list(reader)
        return seeds

    def _write_seeds(self, seeds):
        """Write all seeds to CSV"""
        with open(self.seeds_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['id', 'seed_name', 'planting_depth_cm', 'spacing_cm', 'germination_days', 'description'])
            writer.writerows(seeds)

    def add_seed(self, seed_name, planting_depth_cm, spacing_cm=None, germination_days=None, description=None):
        """Add a new seed type to the catalog"""
        with self._lock:
            seeds = self._read_seeds()
            
            # Find max id and check if seed exists
            max_id = 0
            for i, seed in enumerate(seeds):
                try:
                    max_id = max(max_id, int(seed[0]))
                    if seed[1] == seed_name:
                        # Update existing
                        seeds[i] = [seed[0], seed_name, planting_depth_cm, spacing_cm or '', germination_days or '', description or '']
                        self._write_seeds(seeds)
                        return
                except (ValueError, IndexError):
                    continue
            
            # Add new
            new_id = max_id + 1
            seeds.append([new_id, seed_name, planting_depth_cm, spacing_cm or '', germination_days or '', description or ''])
            self._write_seeds(seeds)

    def get_all_seeds(self):
        """Returns all seeds with their planting info"""
        with self._lock:
            seeds = self._read_seeds()
            result = []
            for seed in seeds:
                try:
                    result.append((
                        int(seed[0]),                              # id
                        seed[1],                                   # seed_name
                        float(seed[2]) if seed[2] else 0.0,       # planting_depth_cm
                        float(seed[3]) if seed[3] else None,      # spacing_cm
                        int(seed[4]) if seed[4] else None,        # germination_days
                        seed[5] if len(seed) > 5 else ''          # description
                    ))
                except (ValueError, IndexError):
                    continue
            return result

    def get_seed_depth(self, seed_name):
        """Get the planting depth for a specific seed type"""
        with self._lock:
            seeds = self._read_seeds()
            for seed in seeds:
                if seed[1] == seed_name:
                    try:
                        return float(seed[2])
                    except (ValueError, IndexError):
                        return None
            return None

    def delete_seed(self, seed_name):
        """Remove a seed type from the catalog"""
        with self._lock:
            seeds = self._read_seeds()
            seeds = [s for s in seeds if s[1] != seed_name]
            self._write_seeds(seeds)

    # ============ Soil Scans Methods ============
    def _read_scans(self):
        """Read all soil scans from CSV"""
        scans = []
        if os.path.exists(self.scans_file):
            with open(self.scans_file, 'r', newline='') as f:
                reader = csv.reader(f)
                next(reader, None)  # Skip header
                scans = list(reader)
        return scans

    def _write_scans(self, scans):
        """Write all soil scans to CSV"""
        with open(self.scans_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['id', 'grid_x', 'grid_y', 'world_x', 'world_y', 'timestamp',
                            'nitrogen_ppm', 'phosphorus_ppm', 'potassium_ppm', 'ph_level',
                            'moisture_percent', 'temperature_celsius', 'ec_mS_cm', 'notes'])
            writer.writerows(scans)

    def add_soil_scan(self, grid_x, grid_y, world_x, world_y, 
                      nitrogen=None, phosphorus=None, potassium=None, ph=None,
                      moisture=None, temperature=None, ec=None, notes=None):
        """Record a soil scan reading at a specific location
        
        7 readings: N, P, K, pH, Moisture, Temperature, Electrical Conductivity
        """
        with self._lock:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            scans = self._read_scans()
            
            # Find max id
            max_id = 0
            for scan in scans:
                try:
                    max_id = max(max_id, int(scan[0]))
                except (ValueError, IndexError):
                    continue
            
            new_id = max_id + 1
            scans.append([
                new_id, grid_x, grid_y, world_x, world_y, timestamp,
                nitrogen or '', phosphorus or '', potassium or '', ph or '',
                moisture or '', temperature or '', ec or '', notes or ''
            ])
            self._write_scans(scans)
            return new_id

    def get_all_soil_scans(self):
        """Returns all soil scan readings
        
        Returns tuples: (id, grid_x, grid_y, world_x, world_y, timestamp,
                         nitrogen, phosphorus, potassium, ph, moisture, temperature, ec, notes)
        """
        with self._lock:
            scans = self._read_scans()
            result = []
            for scan in scans:
                try:
                    result.append((
                        int(scan[0]),                              # id
                        int(scan[1]),                              # grid_x
                        int(scan[2]),                              # grid_y
                        float(scan[3]) if scan[3] else 0.0,       # world_x
                        float(scan[4]) if scan[4] else 0.0,       # world_y
                        scan[5],                                   # timestamp
                        float(scan[6]) if scan[6] else None,      # nitrogen_ppm
                        float(scan[7]) if scan[7] else None,      # phosphorus_ppm
                        float(scan[8]) if scan[8] else None,      # potassium_ppm
                        float(scan[9]) if scan[9] else None,      # ph_level
                        float(scan[10]) if scan[10] else None,    # moisture_percent
                        float(scan[11]) if scan[11] else None,    # temperature_celsius
                        float(scan[12]) if scan[12] else None,    # ec_mS_cm
                        scan[13] if len(scan) > 13 else ''        # notes
                    ))
                except (ValueError, IndexError):
                    continue
            # Sort by timestamp descending
            result.sort(key=lambda x: x[5], reverse=True)
            return result

    def get_soil_scan_for_cell(self, grid_x, grid_y):
        """Get the most recent soil scan for a specific grid cell"""
        scans = self.get_all_soil_scans()
        for scan in scans:
            if scan[1] == grid_x and scan[2] == grid_y:
                return scan
        return None

    def get_soil_scans_in_region(self, min_x, max_x, min_y, max_y):
        """Get all soil scans within a grid region"""
        scans = self.get_all_soil_scans()
        return [s for s in scans if min_x <= s[1] <= max_x and min_y <= s[2] <= max_y]

    def close(self):
        """No-op for CSV (kept for interface compatibility)"""
        pass