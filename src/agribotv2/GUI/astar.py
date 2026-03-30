import math
import heapq

class AStarPlanner:
    def __init__(self, min_x, max_x, min_y, max_y, resolution=0.1, turn_penalty=0.5, min_turn_radius=0.75):
        self.resolution = resolution
        self.min_x = min_x
        self.min_y = min_y
        self.width = int((max_x - min_x) / resolution)
        self.height = int((max_y - min_y) / resolution)
        self.obstacles = set()  # Set of (ix, iy) tuples
        self.turn_penalty = turn_penalty  # Penalty for changing direction (encourages straight paths)
        self.min_turn_radius = min_turn_radius  # Minimum turn radius in meters

    def to_grid(self, x, y):
        ix = int((x - self.min_x) / self.resolution)
        iy = int((y - self.min_y) / self.resolution)
        return ix, iy

    def to_world(self, ix, iy):
        x = self.min_x + (ix * self.resolution)
        y = self.min_y + (iy * self.resolution)
        return x, y

    def add_obstacle(self, x, y, radius):
        """ Adds a circular obstacle to the grid """
        ix, iy = self.to_grid(x, y)
        ir = int(radius / self.resolution)
        
        for dx in range(-ir, ir + 1):
            for dy in range(-ir, ir + 1):
                if dx*dx + dy*dy <= ir*ir:
                    self.obstacles.add((ix + dx, iy + dy))

    def clear_obstacles(self):
        self.obstacles.clear()

    def plan(self, sx, sy, gx, gy):
        start_node = self.to_grid(sx, sy)
        goal_node = self.to_grid(gx, gy)
        
        # Check bounds
        if not (0 <= start_node[0] < self.width and 0 <= start_node[1] < self.height):
            return []  # Start out of bounds
        if not (0 <= goal_node[0] < self.width and 0 <= goal_node[1] < self.height):
            return []  # Goal out of bounds

        open_set = []
        # Store (f_score, node, direction) - direction helps penalize turns
        heapq.heappush(open_set, (0, start_node, None))
        came_from = {}
        g_score = {start_node: 0}
        direction_at = {start_node: None}  # Track direction we arrived from
        
        # 8-Connected Movement: (dx, dy, base_cost)
        motions = [
            (1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        while open_set:
            _, current, prev_direction = heapq.heappop(open_set)

            if current == goal_node:
                raw_path = self._reconstruct_path(came_from, current)
                # Apply path optimization to reduce curves
                optimized_path = self._simplify_path(raw_path)
                optimized_path = self._line_of_sight_optimization(optimized_path)
                # Smooth path to enforce minimum turn radius
                smoothed_path = self._smooth_path_with_radius(optimized_path)
                # Add intermediate points for better resolution
                dense_path = self._densify_path(smoothed_path, spacing=0.15)
                return dense_path

            for dx, dy, base_cost in motions:
                neighbor = (current[0] + dx, current[1] + dy)
                current_direction = (dx, dy)
                
                # Boundary Check
                if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height):
                    continue
                
                # Obstacle Check
                if neighbor in self.obstacles:
                    continue

                # Calculate cost with turn penalty
                cost = base_cost
                if prev_direction is not None and current_direction != prev_direction:
                    # Add penalty for changing direction (turning)
                    cost += self.turn_penalty

                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    direction_at[neighbor] = current_direction
                    f = tentative_g + self.heuristic(neighbor, goal_node)
                    heapq.heappush(open_set, (f, neighbor, current_direction))
                    came_from[neighbor] = current

        return []  # No path found

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _reconstruct_path(self, came_from, current):
        """Reconstruct the raw path from A* search"""
        path = []
        while current in came_from:
            wx, wy = self.to_world(current[0], current[1])
            path.append({'x': wx, 'y': wy})
            current = came_from[current]
        path.reverse()
        return path

    def _simplify_path(self, path):
        """
        Remove collinear waypoints (points on the same line).
        This reduces unnecessary waypoints when the robot is already going straight.
        """
        if len(path) < 3:
            return path
        
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            curr = path[i]
            next_pt = path[i + 1]
            
            # Calculate direction vectors
            dir1 = (curr['x'] - prev['x'], curr['y'] - prev['y'])
            dir2 = (next_pt['x'] - curr['x'], next_pt['y'] - curr['y'])
            
            # Normalize directions (handle zero-length)
            len1 = math.hypot(dir1[0], dir1[1])
            len2 = math.hypot(dir2[0], dir2[1])
            
            if len1 > 0.001 and len2 > 0.001:
                dir1 = (dir1[0] / len1, dir1[1] / len1)
                dir2 = (dir2[0] / len2, dir2[1] / len2)
                
                # Check if directions are similar (cross product close to zero)
                cross = dir1[0] * dir2[1] - dir1[1] * dir2[0]
                
                # If not collinear (cross product > threshold), keep the waypoint
                if abs(cross) > 0.1:
                    simplified.append(curr)
            else:
                simplified.append(curr)
        
        # Always add the last point
        simplified.append(path[-1])
        return simplified

    def _line_of_sight_optimization(self, path):
        """
        Skip intermediate waypoints when there's a clear line of sight.
        This creates longer straight segments for the robot to follow.
        """
        if len(path) < 3:
            return path
        
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Try to find the farthest point we can reach directly
            farthest = i + 1
            
            for j in range(i + 2, len(path)):
                if self._has_line_of_sight(path[i], path[j]):
                    farthest = j
            
            optimized.append(path[farthest])
            i = farthest
        
        return optimized

    def _has_line_of_sight(self, p1, p2):
        """
        Check if there's a clear line of sight between two world-coordinate points.
        Uses Bresenham-style line checking on the grid.
        """
        x0, y0 = self.to_grid(p1['x'], p1['y'])
        x1, y1 = self.to_grid(p2['x'], p2['y'])
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # Check if current cell is an obstacle
            if (x, y) in self.obstacles:
                return False
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True

    def _smooth_path_with_radius(self, path):
        """
        Smooth the path to ensure minimum turn radius of self.min_turn_radius.
        Uses quadratic Bezier curves at corners.
        """
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            p0 = path[i - 1]
            p1 = path[i]  # Corner point
            p2 = path[i + 1]
            
            # Calculate vectors
            v1 = (p1['x'] - p0['x'], p1['y'] - p0['y'])
            v2 = (p2['x'] - p1['x'], p2['y'] - p1['y'])
            
            len1 = math.hypot(v1[0], v1[1])
            len2 = math.hypot(v2[0], v2[1])
            
            if len1 < 0.01 or len2 < 0.01:
                smoothed.append(p1)
                continue
            
            # Normalize
            v1n = (v1[0] / len1, v1[1] / len1)
            v2n = (v2[0] / len2, v2[1] / len2)
            
            # Calculate angle between segments
            dot = v1n[0] * v2n[0] + v1n[1] * v2n[1]
            dot = max(-1, min(1, dot))  # Clamp for acos
            angle = math.acos(dot)
            
            if angle < 0.1:  # Nearly straight, no smoothing needed
                smoothed.append(p1)
                continue
            
            # Calculate offset distance for minimum radius curve
            # For a circular arc: offset = radius * tan(angle/2)
            offset = self.min_turn_radius * math.tan(angle / 2)
            offset = min(offset, len1 * 0.4, len2 * 0.4)  # Don't exceed segment length
            
            # Entry point (before corner)
            entry = {
                'x': p1['x'] - v1n[0] * offset,
                'y': p1['y'] - v1n[1] * offset
            }
            
            # Exit point (after corner)
            exit_pt = {
                'x': p1['x'] + v2n[0] * offset,
                'y': p1['y'] + v2n[1] * offset
            }
            
            # Generate arc points using quadratic Bezier
            smoothed.append(entry)
            num_arc_points = max(3, int(angle * 3))  # More points for sharper turns
            for t in range(1, num_arc_points):
                t_norm = t / num_arc_points
                # Quadratic Bezier: B(t) = (1-t)^2*P0 + 2*(1-t)*t*P1 + t^2*P2
                bx = (1-t_norm)**2 * entry['x'] + 2*(1-t_norm)*t_norm * p1['x'] + t_norm**2 * exit_pt['x']
                by = (1-t_norm)**2 * entry['y'] + 2*(1-t_norm)*t_norm * p1['y'] + t_norm**2 * exit_pt['y']
                smoothed.append({'x': bx, 'y': by})
            smoothed.append(exit_pt)
        
        smoothed.append(path[-1])
        return smoothed

    def _densify_path(self, path, spacing=0.15):
        """
        Add intermediate points along the path for finer control.
        spacing: distance between points in meters
        """
        if len(path) < 2:
            return path
        
        dense = [path[0]]
        
        for i in range(1, len(path)):
            p0 = dense[-1]
            p1 = path[i]
            
            dist = math.hypot(p1['x'] - p0['x'], p1['y'] - p0['y'])
            
            if dist > spacing:
                num_points = int(dist / spacing)
                dx = (p1['x'] - p0['x']) / (num_points + 1)
                dy = (p1['y'] - p0['y']) / (num_points + 1)
                
                for j in range(1, num_points + 1):
                    dense.append({
                        'x': p0['x'] + dx * j,
                        'y': p0['y'] + dy * j
                    })
            
            dense.append(p1)
        
        return dense

    # Keep old method name for backwards compatibility
    def reconstruct_path(self, came_from, current):
        """Backwards compatible wrapper"""
        raw_path = self._reconstruct_path(came_from, current)
        optimized_path = self._simplify_path(raw_path)
        optimized_path = self._line_of_sight_optimization(optimized_path)
        return optimized_path