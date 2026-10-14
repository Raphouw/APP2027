"""Processus graphique de navigation sur carte."""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.widgets import Button
from .map_format import load_map

def plotter_process(shared_data, yaml_path):
    meta, grid_data, _ = load_map(yaml_path)
    res, origin = meta['resolution'], meta['origin']
    H, W = grid_data.shape
    grid_display = np.ones((H, W), dtype=int)
    prob = grid_data/255.0 if meta.get("negate", 0) else (255.0-grid_data)/255.0
    grid_display[prob < meta['free_thresh']] = 0
    grid_display[prob > meta['occupied_thresh']] = 2

    xmin, xmax = origin[0], origin[0] + W * res
    ymin, ymax = origin[1], origin[1] + H * res
    ext = [xmin, xmax, ymin, ymax]
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))
    cmap = mcolors.ListedColormap(['#2ecc71', '#bdc3c7', '#000000', '#e74c3c'])
    
    # --- BOUTONS UI ---
    ax_stop = plt.axes([0.78, 0.02, 0.15, 0.06])
    btn_stop = Button(ax_stop, 'STOP', color='#e74c3c')
    btn_stop.on_clicked(lambda x: shared_data.update({'state': 'STOPPED'}))

    ax_pose = plt.axes([0.58, 0.02, 0.18, 0.06])
    btn_pose = Button(ax_pose, 'SET POSE', color='#3498db')
    btn_pose.on_clicked(lambda x: shared_data.update({'ui_mode': 'SET_POSE'}))

    # --- LOGIQUE CLIC & GLISSE ---
    press_data = {'x': None, 'y': None}

    def on_press(e):
        if e.inaxes != ax: return
        if shared_data.get('ui_mode') == 'SET_POSE':
            press_data['x'] = e.xdata
            press_data['y'] = e.ydata
        else:
            shared_data['click'] = (e.xdata, e.ydata)

    def on_release(e):
        if e.inaxes != ax: return
        if shared_data.get('ui_mode') == 'SET_POSE' and press_data['x'] is not None:
            dx = e.xdata - press_data['x']
            dy = e.ydata - press_data['y']
            # Si l'utilisateur a glissé la souris pour donner un angle
            if math.hypot(dx, dy) > 0.2:
                yaw = math.atan2(dy, dx)
                shared_data['set_pose_cmd'] = (press_data['x'], press_data['y'], yaw)
            else:
                # S'il a juste cliqué sans glisser, on garde l'orientation actuelle
                shared_data['set_pose_cmd'] = (press_data['x'], press_data['y'], None)
            
            shared_data['ui_mode'] = 'GOAL' # On repasse en mode navigation
            press_data['x'] = None

    fig.canvas.mpl_connect('close_event', lambda event: shared_data.update(quit=True, state='STOPPED'))
    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('button_release_event', on_release)

    while True:
        try:
            if shared_data.get('quit'): break
            
            r_pos = shared_data.get('robot_pos', [0.0, 0.0, 0.0])
            path = shared_data.get('path', [])
            lidar_pts = shared_data.get('lidar_pts', [])
            pose_ok = shared_data.get('pose_received', False)
            ui_mode = shared_data.get('ui_mode', 'GOAL')
            
            ax.clear()
            ax.imshow(grid_display, cmap=cmap, extent=ext, origin='lower', vmin=0, vmax=3)
            
            # 🗺️ Origine de la carte (0,0 en coordonnées carte)
            # Note: map origin en world coords, mais on affiche aussi le 0,0 carte
            ox, oy = origin[0], origin[1]
            # Axes de coordonnées
            ax.arrow(ox, oy, 0.8, 0, head_width=0.06, color='#FF6B35', label="Monde X", linewidth=2)
            ax.arrow(ox, oy, 0, 0.8, head_width=0.06, color='#00CC96', label="Monde Y", linewidth=2)
            # Marqueur origine (petit carré blanc)
            ax.plot(ox, oy, 's', color='white', markersize=8, markeredgecolor='black', markeredgewidth=1.5, zorder=8, label="Origine (0,0)")

            if lidar_pts:
                lx, ly = zip(*lidar_pts)
                ax.scatter(lx, ly, s=3, c='#e74c3c', alpha=0.8)
            
            new_obstacles = shared_data.get('new_obstacles', [])
            if new_obstacles:
                nx, ny = zip(*new_obstacles)
                ax.scatter(nx, ny, s=22, marker='s', c='#e0218a', zorder=6, label='Nouvel obstacle')

            if path:
                px, py = zip(*([tuple(r_pos[:2])]+list(path)))
                ax.plot(px, py, 'b-', lw=2)
            
            rx, ry, r_yaw = r_pos
            # ⭐ Robot : gros point bleu + flèche de direction
            ax.plot(rx, ry, 'o', color='#2E86AB', markersize=16, zorder=10, label=f"Robot ({rx:.2f}, {ry:.2f})")
            # Flèche direction principale (rouge)
            ax.arrow(rx, ry, 0.35 * math.cos(r_yaw), 0.35 * math.sin(r_yaw),
                     head_width=0.08, head_length=0.08, color='#FF6B35', linewidth=2, zorder=9)
            # Flèche perpendiculaire pour visualisation (vert)
            ax.arrow(rx, ry, 0.25 * math.cos(r_yaw + math.pi/2), 0.25 * math.sin(r_yaw + math.pi/2),
                     head_width=0.06, head_length=0.06, color='#00CC96', linewidth=1.5, zorder=9)
            
            if shared_data.get('goal'):
                gx, gy = shared_data['goal']
                ax.plot(gx, gy, 'rx', markersize=12, markeredgewidth=3)
            
            ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
            
            odom_warn = "" if pose_ok else " ⚠️ ODOM INACTIF"
            mode_text = "🎯 Clic=Destination" if ui_mode == 'GOAL' else "📐 Clic & Glisse=Position"
            ax.set_title(f"[{mode_text}] | Etat: {shared_data.get('state')} | Yaw: {math.degrees(r_yaw):.0f}°{odom_warn}")
            
            handles, labels = ax.get_legend_handles_labels()
            if handles: ax.legend(handles, labels, loc="upper right")
            
            plt.draw(); plt.pause(0.02)
        except Exception as e:
            print(f"Erreur d'affichage : {e}")
            break
