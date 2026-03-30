"""
Fast Path Planner using optimized A* with numpy for speed.
Designed for real-time path planning with minimum turn radius constraints.
Max turn angle: 25 degrees for smooth robot-friendly curves.
"""
import math
import numpy as np
from collections import deque
import heapq

class FastPlanner:
    # Robot front offset from center (meters)
    ROBOT_FRONT_OFFSET = 1
    # Maximum turn angle in radians (25 degrees)
    MAX_TURN_ANGLE = math.radians(25)
    
    def __init__(self, min_x, max_x, min_y, max_y, resolution=0.15, robot_radius=0.6, min_turn_radius=0.75):
        self.resolution = resolution
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.robot_radius = robot_radius
        self.min_turn_radius = min_turn_radius
        
        self.width = int((max_x - min_x) / resolution) + 1
        self.height = int((max_y - min_y) / resolution) + 1
        
        # Use numpy array for faster obstacle checking
        self.obstacle_grid = np.zeros((self.width, self.height), dtype=np.uint8)
        
        # Precompute motion primitives
        self.motions = np.array([
            [1, 0, 1.0],      # Right
            [-1, 0, 1.0],     # Left
            [0, 1, 1.0],      # Up
            [0, -1, 1.0],     # Down
            [1, 1, 1.414],    # Diagonal
            [1, -1, 1.414],
            [-1, 1, 1.414],
            [-1, -1, 1.414]
        ])
        
    def to_grid(self, x, y):
        ix = int((x - self.min_x) / self.resolution)
        iy = int((y - self.min_y) / self.resolution)
        return max(0, min(ix, self.width-1)), max(0, min(iy, self.height-1))
    
    def to_world(self, ix, iy):
        return self.min_x + ix * self.resolution, self.min_y + iy * self.resolution
    
    def add_obstacle(self, x, y, radius):
        """Add circular obstacle with inflation for robot radius"""
        total_radius = radius + self.robot_radius
        ix, iy = self.to_grid(x, y)
        ir = int(total_radius / self.resolution) + 1
        
        for dx in range(-ir, ir + 1):
            for dy in range(-ir, ir + 1):
                if dx*dx + dy*dy <= ir*ir:
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.obstacle_grid[nx, ny] = 1
    
    def clear_obstacles(self):
        self.obstacle_grid.fill(0)
    
    def _find_nearest_free(self, gx, gy, max_search=10):
        """Find nearest unblocked cell to the goal within search radius."""
        for radius in range(1, max_search + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:  # Only check perimeter
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.width and 0 <= ny < self.height:
                            if not self.obstacle_grid[nx, ny]:
                                return (nx, ny)
        return None
    
    def plan(self, sx, sy, gx, gy, robot_yaw=None):
        """
        Fast A* with numpy optimization.
        If robot_yaw is provided, path starts from robot front point.
        """        
        start = self.to_grid(sx, sy)
        goal = self.to_grid(gx, gy)
        
        # Clear the start cell and cells around robot - robot is already there
        # Clear the start cell and cells around robot - robot is already there
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                nx, ny = start[0] + dx, start[1] + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.obstacle_grid[nx, ny] = 0
        
        # If goal is blocked, try to find nearest unblocked cell
        if self.obstacle_grid[goal[0], goal[1]]:
            new_goal = self._find_nearest_free(goal[0], goal[1])
            if new_goal is None:
                return []
            goal = new_goal
        
        # A* with priority queue
        open_set = [(0, start[0], start[1], -1)]  # (f_score, x, y, prev_direction)
        g_score = np.full((self.width, self.height), np.inf)
        g_score[start[0], start[1]] = 0
        came_from = {}
        visited = set()
        
        while open_set:
            _, cx, cy, prev_dir = heapq.heappop(open_set)
            
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))
            
            if (cx, cy) == goal:
                # Reconstruct path
                path = self._reconstruct(came_from, goal)
                # Optimize and smooth
                path = self._optimize_path(path)
                return path
            
            for i, (dx, dy, cost) in enumerate(self.motions):
                nx, ny = int(cx + dx), int(cy + dy)
                
                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue
                if self.obstacle_grid[nx, ny]:
                    continue
                
                # Add turn penalty
                turn_cost = 0.3 if prev_dir >= 0 and i != prev_dir else 0
                
                new_g = g_score[cx, cy] + cost + turn_cost
                
                if new_g < g_score[nx, ny]:
                    g_score[nx, ny] = new_g
                    h = math.hypot(nx - goal[0], ny - goal[1])
                    f = new_g + h
                    heapq.heappush(open_set, (f, nx, ny, i))
                    came_from[(nx, ny)] = (cx, cy)
        
        return []  # No path found
    
    def _reconstruct(self, came_from, goal):
        """Reconstruct path from A* result"""
        path = []
        current = goal
        while current in came_from:
            wx, wy = self.to_world(current[0], current[1])
            path.append({'x': wx, 'y': wy})
            current = came_from[current]
        wx, wy = self.to_world(current[0], current[1])
        path.append({'x': wx, 'y': wy})
        path.reverse()
        return path
    
    def _optimize_path(self, path):
        """Line-of-sight optimization followed by smoothing"""
        if len(path) < 3:
            return path
        
        # Line of sight optimization
        optimized = [path[0]]
        i = 0
        while i < len(path) - 1:
            farthest = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self._line_of_sight(path[i], path[j]):
                    farthest = j
                    break
            optimized.append(path[farthest])
            i = farthest
        
        # Smooth corners with arc
        smoothed = self._smooth_corners(optimized)

        # Enforce heading changes <= 30 degrees between consecutive segments
        capped = self._enforce_max_heading_change(smoothed, max_angle=math.radians(30))
        
        # Densify for tracking while keeping waypoint count modest
        return self._densify(capped, spacing=0.25)
    
    def _line_of_sight(self, p1, p2):
        """Fast line of sight check using numpy"""
        x0, y0 = self.to_grid(p1['x'], p1['y'])
        x1, y1 = self.to_grid(p2['x'], p2['y'])
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            if self.obstacle_grid[x, y]:
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def _smooth_corners(self, path):
        """
        Smooth corners with maximum turn angle constraint (25 degrees).
        Creates gentle curves that are easy for the robot to follow.
        """
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            p0, p1, p2 = path[i-1], path[i], path[i+1]
            
            v1 = (p1['x'] - p0['x'], p1['y'] - p0['y'])
            v2 = (p2['x'] - p1['x'], p2['y'] - p1['y'])
            
            len1 = math.hypot(v1[0], v1[1])
            len2 = math.hypot(v2[0], v2[1])
            
            if len1 < 0.01 or len2 < 0.01:
                smoothed.append(p1)
                continue
            
            v1n = (v1[0]/len1, v1[1]/len1)
            v2n = (v2[0]/len2, v2[1]/len2)
            
            dot = max(-1, min(1, v1n[0]*v2n[0] + v1n[1]*v2n[1]))
            angle = math.acos(dot)
            
            # If angle is small, just add the point
            if angle < 0.1:
                smoothed.append(p1)
                continue
            
            # For larger turns, create smooth curve with max 25 degree increments
            # Calculate how many curve segments we need
            num_segments = max(3, int(math.ceil(angle / self.MAX_TURN_ANGLE)))
            
            # Calculate arc offset based on turn radius
            offset = min(self.min_turn_radius * math.tan(angle/2), len1*0.45, len2*0.45)
            
            entry = {'x': p1['x'] - v1n[0]*offset, 'y': p1['y'] - v1n[1]*offset}
            exit_pt = {'x': p1['x'] + v2n[0]*offset, 'y': p1['y'] + v2n[1]*offset}
            
            smoothed.append(entry)
            
            # Generate bezier curve points for smooth transition
            # More points for sharper turns to keep each segment within 25 degrees
            for j in range(1, num_segments):
                t = j / num_segments
                # Quadratic bezier curve
                bx = (1-t)**2*entry['x'] + 2*(1-t)*t*p1['x'] + t**2*exit_pt['x']
                by = (1-t)**2*entry['y'] + 2*(1-t)*t*p1['y'] + t**2*exit_pt['y']
                smoothed.append({'x': bx, 'y': by})
            
            smoothed.append(exit_pt)
        
        smoothed.append(path[-1])
        return smoothed

    def _enforce_max_heading_change(self, path, max_angle):
        """Ensure turn angles between consecutive segments do not exceed max_angle (radians)."""
        if len(path) < 3:
            return path

        pts = path[:]
        i = 1
        while i < len(pts) - 1:
            p0, p1, p2 = pts[i-1], pts[i], pts[i+1]
            v1 = (p1['x'] - p0['x'], p1['y'] - p0['y'])
            v2 = (p2['x'] - p1['x'], p2['y'] - p1['y'])
            len1 = math.hypot(v1[0], v1[1]); len2 = math.hypot(v2[0], v2[1])
            if len1 < 1e-3 or len2 < 1e-3:
                i += 1
                continue

            v1n = (v1[0]/len1, v1[1]/len1)
            v2n = (v2[0]/len2, v2[1]/len2)
            dot = max(-1.0, min(1.0, v1n[0]*v2n[0] + v1n[1]*v2n[1]))
            angle = math.acos(dot)
            if angle <= max_angle + 1e-3:
                i += 1
                continue

            # Rotate v1 toward v2 by max_angle to create an intermediate heading
            cross = v1n[0]*v2n[1] - v1n[1]*v2n[0]
            sign = 1.0 if cross >= 0 else -1.0
            a = max_angle * sign
            cos_a, sin_a = math.cos(a), math.sin(a)
            new_dir = (v1n[0]*cos_a - v1n[1]*sin_a, v1n[0]*sin_a + v1n[1]*cos_a)

            step = min(len2, self.min_turn_radius, 0.6 * len1)
            new_pt = {'x': p1['x'] + new_dir[0]*step, 'y': p1['y'] + new_dir[1]*step}

            pts.insert(i+1, new_pt)
            # Re-evaluate with the new point in place
        return pts
    
    def _densify(self, path, spacing=0.12):
        """Add intermediate points for better tracking"""
        if len(path) < 2:
            return path
        
        dense = [path[0]]
        for i in range(1, len(path)):
            p0, p1 = dense[-1], path[i]
            dist = math.hypot(p1['x']-p0['x'], p1['y']-p0['y'])
            
            if dist > spacing:
                n = int(dist / spacing)
                for j in range(1, n+1):
                    t = j / (n+1)
                    dense.append({
                        'x': p0['x'] + t*(p1['x']-p0['x']),
                        'y': p0['y'] + t*(p1['y']-p0['y'])
                    })
            dense.append(p1)
        
        return dense


class ObstacleAvoidanceController:
    """
    Smart obstacle avoidance with directional rotation logic.
    - First rotates left when obstacle detected
    - Only rotates right if left is blocked after 60 degrees
    """
    
    def __init__(self):
        self.avoidance_state = "NONE"  # NONE, ROTATING_LEFT, ROTATING_RIGHT, CLEAR
        self.start_yaw = 0
        self.total_rotation = 0
        self.obstacle_detected_at = None
        
    def update(self, robot_yaw, us_front_left, us_front_right, us_left, us_right, obstacle_dist=2.5):
        """
        Update obstacle avoidance state machine.
        
        Returns: (action, angular_vel)
            action: 'CONTINUE', 'ROTATE', 'REVERSE', 'REPLAN'
            angular_vel: rotation velocity if action is 'ROTATE'
        """
        front_min = min(us_front_left, us_front_right)
        
        if self.avoidance_state == "NONE":
            if front_min < obstacle_dist:
                # Start rotating left first
                self.avoidance_state = "ROTATING_LEFT"
                self.start_yaw = robot_yaw
                self.total_rotation = 0
                self.obstacle_detected_at = front_min
                return ('ROTATE', 0.8)  # Rotate left (positive angular vel)
            return ('CONTINUE', 0)
        
        elif self.avoidance_state == "ROTATING_LEFT":
            # Calculate how much we've rotated
            yaw_diff = self._normalize_angle(robot_yaw - self.start_yaw)
            self.total_rotation = abs(yaw_diff)
            
            # Check if front is now clear
            if front_min >= obstacle_dist + 0.5:
                self.avoidance_state = "CLEAR"
                return ('REPLAN', 0)
            
            # If rotated 60+ degrees but left sensor also sees obstacle, try right
            if self.total_rotation >= math.radians(60):
                if us_left < 1.5:  # Left is also blocked
                    self.avoidance_state = "ROTATING_RIGHT"
                    self.start_yaw = robot_yaw
                    self.total_rotation = 0
                    return ('ROTATE', -0.8)  # Rotate right
                else:
                    # Left is clear, keep rotating left
                    return ('ROTATE', 0.8)
            
            return ('ROTATE', 0.8)
        
        elif self.avoidance_state == "ROTATING_RIGHT":
            yaw_diff = self._normalize_angle(robot_yaw - self.start_yaw)
            self.total_rotation = abs(yaw_diff)
            
            if front_min >= obstacle_dist + 0.5:
                self.avoidance_state = "CLEAR"
                return ('REPLAN', 0)
            
            # Rotated 120 degrees total (60 left + 60 right), need to reverse
            if self.total_rotation >= math.radians(60):
                self.avoidance_state = "NONE"
                return ('REVERSE', 0)
            
            return ('ROTATE', -0.8)
        
        elif self.avoidance_state == "CLEAR":
            self.avoidance_state = "NONE"
            return ('REPLAN', 0)
        
        return ('CONTINUE', 0)
    
    def reset(self):
        self.avoidance_state = "NONE"
        self.total_rotation = 0
    
    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
