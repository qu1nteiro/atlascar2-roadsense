"""Figures for the report and the presentation, from a bag (no ROS needed).

    pip install mcap mcap-ros2-support matplotlib numpy pillow
    python tools/figura_bag.py data/<bag_folder> --grupo g03
    python tools/figura_bag.py data/<bag_folder> --grupo g03 --gif docs/img/demo.gif

Writes to docs/img/:
  <bag>_instantes.png   grid + forbidden zone + path + goal at 3 snapshots
  <bag>_estado.png      planner state and lateral deviation over time
  (optional) animated GIF with the full sequence
"""
import argparse
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import PillowWriter
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from mcap_ros2.reader import read_ros2_messages

matplotlib.use('Agg')     # only writes files, never opens windows


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(bag, group):
    mcap = next(bag.glob('*.mcap')) if bag.is_dir() else bag
    t = {k: f'/{group}/{k}' for k in ('grelha_local', 'zona_proibida', 'caminho_local',
                                      'estado', 'objetivo')}
    grids, zones, paths, states, goals = {}, {}, {}, [], []
    for m in read_ros2_messages(str(mcap), topics=list(t.values())):
        msg = m.ros_msg
        if m.channel.topic == t['grelha_local']:
            grids[stamp(msg)] = msg
        elif m.channel.topic == t['zona_proibida']:
            zones[stamp(msg)] = msg
        elif m.channel.topic == t['caminho_local']:
            paths[stamp(msg)] = msg
        elif m.channel.topic == t['objetivo']:
            goals.append((m.log_time_ns * 1e-9, (msg.pose.position.x, msg.pose.position.y)))
        else:
            states.append((m.log_time_ns * 1e-9, msg.data))
    # one "snapshot" = path + grid + forbidden zone with the same header.stamp
    snapshots = [(s, grids[s], zones.get(s), paths[s])
                 for s in sorted(paths) if s in grids]
    return snapshots, states, goals


def draw(ax, grid, zone, path, title, goal=None):
    info = grid.info
    w, h, r = info.width, info.height, info.resolution
    x0, y0 = info.origin.position.x, info.origin.position.y
    ext = [x0, x0 + w * r, y0, y0 + h * r]
    d = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    img = np.where(d < 0, 1, np.where(d >= 50, 2, 0))       # 0 free, 1 unknown, 2 occupied
    ax.imshow(img, origin='lower', extent=ext, interpolation='nearest',
              cmap=ListedColormap(['#ffffff', '#d9dde3', '#1f2328']), vmin=0, vmax=2)
    if zone is not None:
        c = np.asarray(zone.data, dtype=np.int16).reshape(h, w)
        forb = np.ma.masked_where((c < 100) | (d >= 50), c)
        ax.imshow(forb, origin='lower', extent=ext, interpolation='nearest',
                  cmap=ListedColormap(['#f4a3a3']), alpha=0.8)
    xs = [p.pose.position.x for p in path.poses]
    ys = [p.pose.position.y for p in path.poses]
    ax.plot(xs, ys, color='#4a7c00', lw=2.5)
    ax.add_patch(Rectangle((-0.6, -0.75), 3.5 + 0.6, 1.5, fc='#91d400', ec='#4a7c00'))
    if goal is not None:
        ax.plot(*goal, marker='X', ms=11, color='#b91c1c', mec='white', mew=1.2, zorder=5)
        ax.annotate('goal', goal, xytext=(0, 9), textcoords='offset points',
                    ha='center', fontsize=8.5, color='#b91c1c')
    ax.set_title(title, fontsize=10)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')


def state_at(states, t):
    previous = [e for (te, e) in states if te <= t + 0.05]
    return previous[-1] if previous else ''


def goal_at(goals, t, default):
    """Last goal published on /gNN/objetivo up to t; otherwise the parameter one."""
    previous = [o for (to, o) in goals if to <= t + 0.05]
    return previous[-1] if previous else default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('--grupo', default='g03')
    ap.add_argument('--saida', type=Path, default=Path('docs/img'))
    ap.add_argument('--gif', type=Path, help='GIF file to create (optional)')
    ap.add_argument('--objetivo', type=float, nargs=2, default=[25.0, 0.0], metavar=('X', 'Y'),
                    help='local goal (m) if the bag has no /gNN/objetivo; '
                         'same as goal_x/goal_y in params.yaml')
    a = ap.parse_args()
    a.saida.mkdir(parents=True, exist_ok=True)

    snapshots, states, goals = read_bag(a.bag, a.grupo)
    if not snapshots:
        raise SystemExit(f'no /{a.grupo}/caminho_local + /{a.grupo}/grelha_local in the bag')
    t0 = snapshots[0][0]
    name = a.bag.name

    # 1. three snapshots: the first LIVRE, the largest DESVIO and a BLOQUEADO (if any)
    def deviation(e):
        return float(e.split()[1]) if e.startswith('DESVIO') else -1.0
    rot = [(s, g, c, p, state_at(states, s)) for s, g, c, p in snapshots]
    chosen = [next((x for x in rot if x[4] == 'LIVRE'), rot[0]),
              max(rot, key=lambda x: deviation(x[4])),
              next((x for x in rot if x[4] == 'BLOQUEADO'), rot[-1])]
    fig, axs = plt.subplots(len(chosen), 1, figsize=(9, 3.2 * len(chosen)))
    for ax, (s, g, c, p, e) in zip(np.atleast_1d(axs), chosen):
        draw(ax, g, c, p, f't = {s - t0:5.1f} s   {e}',
             goal_at(goals, s, tuple(a.objetivo)))
    fig.tight_layout()
    fig.savefig(a.saida / f'{name}_instantes.png', dpi=150)

    # 2. state and deviation over time
    te = np.array([t for t, _ in states]) - t0
    cod = {'LIVRE': 0, 'DESVIO': 1, 'BLOQUEADO': 2}
    ce = np.array([cod[e.split()[0]] for _, e in states])
    dv = np.array([max(deviation(e), 0.0) for _, e in states])
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 4.5), sharex=True)
    a1.step(te, ce, where='post', color='#1f2328')
    a1.set_yticks([0, 1, 2], ['LIVRE', 'DESVIO', 'BLOQUEADO'])
    a1.grid(True, alpha=0.3)
    a2.plot(te, dv, color='#4a7c00')
    a2.set_ylabel('lateral deviation (m)')
    a2.set_xlabel('time (s)')
    a2.grid(True, alpha=0.3)
    pct = {k: 100.0 * np.mean(ce == v) for k, v in cod.items()}
    a1.set_title('  '.join(f'{k}: {v:.0f}%' for k, v in pct.items()), fontsize=10)
    fig.tight_layout()
    fig.savefig(a.saida / f'{name}_estado.png', dpi=150)
    print(f'{len(snapshots)} snapshots; states: ' +
          ', '.join(f'{k} {v:.0f}%' for k, v in pct.items()))

    # 3. GIF (every 2nd snapshot, to keep it light)
    if a.gif:
        a.gif.parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(8, 4))
        writer = PillowWriter(fps=5)
        with writer.saving(fig, str(a.gif), dpi=80):
            for s, g, c, p, e in rot[::2]:
                ax.clear()
                draw(ax, g, c, p, f't = {s - t0:5.1f} s   {e}',
                     goal_at(goals, s, tuple(a.objetivo)))
                writer.grab_frame()
        print(f'GIF: {a.gif}')


if __name__ == '__main__':
    main()
