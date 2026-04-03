"""Plotly map rendering for the AgriBot dashboard."""
from __future__ import annotations

import math

from .config import (
    BATCH_COLOURS,
    CELL_H,
    CELL_W,
    GRID_CONFIG,
    PLANT_CELL_SIZE,
    PLANT_STATE_COLOURS,
    ROBOT_LENGTH,
    ROBOT_WIDTH,
    SENSOR_OFFSETS,
    STATE_COLOURS,
    US_FOV,
    US_MAX_RANGE,
    WORLD_MAX_X,
    WORLD_MAX_Y,
    WORLD_MIN_X,
    WORLD_MIN_Y,
    get_batch_colour,
)


def build_map_figure(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    us_vals: dict,
    path_pts: list,
    goal_x: float,
    goal_y: float,
    has_goal: bool,
    plants: list[dict],
    selection_area: tuple | None,
    mission_waypoints: list[tuple],
    mission_wp_idx: int,
    cells: list[tuple],
):
    import plotly.graph_objects as go

    fig = go.Figure()
    fig.update_layout(
        xaxis=dict(range=[WORLD_MIN_X, WORLD_MAX_X], constrain='domain', scaleanchor='y', scaleratio=1,
                   showgrid=False, zeroline=False, tickfont=dict(color='#4fc3f7', size=10),
                   title=dict(text='X (m)', font=dict(color='#4fc3f7', size=11))),
        yaxis=dict(range=[WORLD_MIN_Y, WORLD_MAX_Y], constrain='domain', showgrid=False, zeroline=False,
                   tickfont=dict(color='#4fc3f7', size=10),
                   title=dict(text='Y (m)', font=dict(color='#4fc3f7', size=11))),
        margin=dict(l=42, r=8, t=10, b=42),
        plot_bgcolor='#111827',
        paper_bgcolor='#0d1117',
        showlegend=False,
        dragmode='pan',
        height=500,
    )

    x = WORLD_MIN_X
    while x <= WORLD_MAX_X + 1e-6:
        is_major = abs(round(x) - x) < 1e-4
        fig.add_shape(type='line', x0=x, x1=x, y0=WORLD_MIN_Y, y1=WORLD_MAX_Y,
                      line=dict(color='#1e3a5f' if is_major else '#121c2e', width=0.9 if is_major else 0.35))
        x = round(x + 0.5, 2)

    y = WORLD_MIN_Y
    while y <= WORLD_MAX_Y + 1e-6:
        is_major = abs(round(y) - y) < 1e-4
        fig.add_shape(type='line', x0=WORLD_MIN_X, x1=WORLD_MAX_X, y0=y, y1=y,
                      line=dict(color='#1e3a5f' if is_major else '#121c2e', width=0.9 if is_major else 0.35))
        y = round(y + 0.5, 2)

    plotted: set[tuple[float, float]] = set()
    for plant in plants:
        px, py = float(plant['x']), float(plant['y'])
        cx = math.floor(px / PLANT_CELL_SIZE) * PLANT_CELL_SIZE + PLANT_CELL_SIZE / 2
        cy = math.floor(py / PLANT_CELL_SIZE) * PLANT_CELL_SIZE + PLANT_CELL_SIZE / 2
        key = (round(cx, 3), round(cy, 3))
        if key in plotted:
            continue
        plotted.add(key)
        state = plant.get('state', 'not_planted')
        fill = (
            PLANT_STATE_COLOURS['detected']
            if state == 'detected'
            else PLANT_STATE_COLOURS['not_planted']
            if state == 'not_planted'
            else get_batch_colour(plant.get('batch_id', ''))
        )
        half = PLANT_CELL_SIZE / 2
        fig.add_shape(type='rect', x0=cx - half, y0=cy - half, x1=cx + half, y1=cy + half,
                      line=dict(color=fill, width=1.5), fillcolor=fill, opacity=0.45)
        fig.add_trace(go.Scatter(
            x=[cx], y=[cy], mode='markers',
            marker=dict(size=8, color=fill, symbol='square', line=dict(color='white', width=1)),
            hovertext=(f"<b>{plant.get('plant_id', '')}</b>  batch: {plant.get('batch_id', '')}"
                       f"<br>state: {state}  seed: {plant.get('seed_id', '')}"),
            hoverinfo='text', showlegend=False,
        ))

    for gx, gy, cell_state, *_ in cells:
        colour = STATE_COLOURS.get(cell_state, STATE_COLOURS['empty'])
        x0 = GRID_CONFIG['min_x'] + gx * CELL_W
        y0 = GRID_CONFIG['min_y'] + gy * CELL_H
        fig.add_shape(type='rect', x0=x0, y0=y0, x1=x0 + CELL_W, y1=y0 + CELL_H,
                      line=dict(color='#556', width=1), fillcolor=colour, opacity=0.25)

    if selection_area:
        ax0, ay0, ax1, ay1 = selection_area
        fig.add_shape(type='rect', x0=ax0, y0=ay0, x1=ax1, y1=ay1,
                      line=dict(color='#00dcff', width=1.5, dash='dot'),
                      fillcolor='rgba(0,220,255,0.06)')

    if len(path_pts) > 1:
        xs, ys = zip(*path_pts)
        fig.add_trace(go.Scatter(x=list(xs), y=list(ys), mode='lines+markers',
                                 line=dict(color='#00e676', width=1.5, dash='dot'),
                                 marker=dict(size=3, color='#00e676'), hoverinfo='skip', showlegend=False))

    for i, (wx, wy) in enumerate(mission_waypoints):
        done = i < mission_wp_idx
        active = i == mission_wp_idx
        colour = '#555' if done else ('#00ff88' if active else '#00bcd4')
        fig.add_trace(go.Scatter(x=[wx], y=[wy], mode='markers+text',
                                 marker=dict(size=11 if active else 8, color=colour,
                                             line=dict(color='white', width=2 if active else 1)),
                                 text=[f'WP{i + 1}'], textposition='top right',
                                 textfont=dict(color=colour, size=10),
                                 hovertext=f'WP{i + 1}  ({wx:.2f}, {wy:.2f})',
                                 hoverinfo='text', showlegend=False))

    if has_goal:
        fig.add_trace(go.Scatter(x=[goal_x], y=[goal_y], mode='markers',
                                 marker=dict(symbol='x-thin', size=18, color='#ff4444',
                                             line=dict(width=3, color='#ff4444')),
                                 hovertext=f'Goal ({goal_x:.1f}, {goal_y:.1f})',
                                 hoverinfo='text', showlegend=False))

    cone_colours = ['rgba(255,140,0,0.22)', 'rgba(255,80,0,0.22)', 'rgba(255,200,0,0.22)', 'rgba(200,110,0,0.22)']
    for i, (key, offs) in enumerate(SENSOR_OFFSETS.items()):
        d = us_vals.get(key, US_MAX_RANGE)
        d = US_MAX_RANGE if d is None or not math.isfinite(d) else min(float(d), US_MAX_RANGE)
        cos_r, sin_r = math.cos(robot_yaw), math.sin(robot_yaw)
        sx = robot_x + offs['x'] * cos_r - offs['y'] * sin_r
        sy = robot_y + offs['x'] * sin_r + offs['y'] * cos_r
        sensor_yaw = robot_yaw + offs['yaw']
        n = 14
        angles = [sensor_yaw - US_FOV / 2 + j * US_FOV / n for j in range(n + 1)]
        arc_x = [sx] + [sx + d * math.cos(a) for a in angles] + [sx]
        arc_y = [sy] + [sy + d * math.sin(a) for a in angles] + [sy]
        cc = cone_colours[i % len(cone_colours)]
        fig.add_trace(go.Scatter(x=arc_x, y=arc_y, mode='lines', fill='toself', fillcolor=cc,
                                 line=dict(color=cc.replace('0.22', '0.55'), width=0.7),
                                 hoverinfo='skip', showlegend=False))

    hl, hw = ROBOT_LENGTH / 2, ROBOT_WIDTH / 2
    ca, sa = math.cos(robot_yaw), math.sin(robot_yaw)
    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)]
    rxs = [robot_x + cx * ca - cy * sa for cx, cy in corners]
    rys = [robot_y + cx * sa + cy * ca for cx, cy in corners]
    fig.add_trace(go.Scatter(x=rxs, y=rys, mode='lines', fill='toself', line=dict(color='#4fc3f7', width=2),
                             fillcolor='rgba(21,101,192,0.45)', hoverinfo='skip', showlegend=False))

    ax_tip = robot_x + hl * ca
    ay_tip = robot_y + hl * sa
    fig.add_trace(go.Scatter(x=[robot_x, ax_tip], y=[robot_y, ay_tip], mode='lines+markers',
                             marker=dict(symbol='arrow', size=13, angleref='previous', color='#FFD700'),
                             line=dict(color='#FFD700', width=2), hoverinfo='skip', showlegend=False))

    fig.add_trace(go.Scatter(x=[robot_x], y=[robot_y], mode='markers', marker=dict(size=6, color='#ff4444'),
                             hovertext=f'Robot  x={robot_x:.2f}  y={robot_y:.2f}  yaw={math.degrees(robot_yaw):.0f}°',
                             hoverinfo='text', showlegend=False))

    click_x = [WORLD_MIN_X + i * (WORLD_MAX_X - WORLD_MIN_X) / 49 for i in range(50)]
    click_y = [WORLD_MIN_Y + i * (WORLD_MAX_Y - WORLD_MIN_Y) / 49 for i in range(50)]
    fig.add_trace(go.Heatmap(z=[[0] * 50 for _ in range(50)], x=click_x, y=click_y,
                             showscale=False, opacity=0.01,
                             colorscale=[[0, '#111827'], [1, '#111827']],
                             hovertemplate='x=%{x:.2f}  y=%{y:.2f}<extra></extra>'))

    return fig


def legend_rows() -> list[tuple[str, str]]:
    rows = [('#808080', 'Not planted'), ('#FFD700', 'Detected')]
    rows.extend((color, f'Batch {i + 1}') for i, color in enumerate(BATCH_COLOURS[:5]))
    return rows
