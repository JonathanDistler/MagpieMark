"""Read-only desktop telemetry dashboard, run alongside Gazebo."""
import argparse
from collections import deque
import json
import math
import time


def main(argv=None):
    import tkinter as tk
    from tkinter import ttk
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import BatteryStatus, VehicleAttitude, VehicleLocalPosition, VehicleStatus
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n', type=int, default=1, help='Number of drone namespaces')
    args, ros_args = parser.parse_known_args(argv)
    if not 1 <= args.n <= 250:
        parser.error('--n must be between 1 and 250')
    rclpy.init(args=ros_args)
    node = Node('telemetry_dashboard')
    root = tk.Tk()
    root.title('Flight deck • swarm telemetry')
    root.geometry('1180x840')
    root.minsize(900, 680)
    root.configure(background='#0b121b')
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('.', font=('DejaVu Sans', 11))
    style.configure('TFrame', background='#0b121b')
    style.configure('TLabel', background='#0b121b', foreground='#dce8f5')
    style.configure('Card.TFrame', background='#152332')
    style.configure('Card.TLabel', background='#152332', foreground='#8fa6bc')
    style.configure('Value.TLabel', background='#152332', foreground='#6de5ed',
                    font=('DejaVu Sans Mono', 22, 'bold'))
    data = [{} for _ in range(args.n)]
    history = [deque(maxlen=400) for _ in range(args.n)]
    def receive(i, key, msg):
        data[i][key] = (msg, time.monotonic())
        if key == 'state':
            p = msg.pose.pose.position
            history[i].append((p.x, p.y))
    for i in range(args.n):
        for key, kind in [('state', Odometry), ('battery', BatteryStatus),
                          ('attitude', VehicleAttitude), ('vehicle_status', VehicleStatus),
                          ('local_telemetry', VehicleLocalPosition), ('control_state', String)]:
            node.create_subscription(kind, f'/drone_{i}/{key}',
                                     lambda msg, i=i, key=key: receive(i, key, msg), 10)
    style.configure('Deck.TRadiobutton', background='#152332', foreground='#a8bdd0',
                    padding=(14, 10), indicatoron=False, relief='flat')
    style.map('Deck.TRadiobutton', background=[('selected', '#245164'), ('active', '#20384a')],
              foreground=[('selected', '#82f5ef')])
    style.configure('TNotebook', background='#0b121b', borderwidth=0)
    style.configure('TNotebook.Tab', background='#152332', foreground='#a8bdd0', padding=(14, 8))
    style.map('TNotebook.Tab', background=[('selected', '#245164')],
              foreground=[('selected', '#82f5ef')])
    selected = tk.StringVar(value='all')
    colors = ('#61e5ed', '#ffbe66', '#b6a1ff', '#81e6a2', '#ff8da8', '#80baff')
    bar = ttk.Frame(root, padding=(18, 16))
    bar.pack(fill='x')
    ttk.Label(bar, text='FLIGHT DECK', font=('DejaVu Sans', 20, 'bold')).pack(side='left')
    ttk.Label(bar, text='SWARM TELEMETRY  /  LOCAL NED',
              foreground='#8fa6bc').pack(side='right')
    # Horizontally scrollable selector keeps every aircraft accessible in large fleets.
    selector = tk.Canvas(root, height=48, background='#0b121b', highlightthickness=0)
    selector.pack(fill='x', padx=16)
    selector_scroll = ttk.Scrollbar(root, orient='horizontal', command=selector.xview)
    selector_scroll.pack(fill='x', padx=16)
    selector.configure(xscrollcommand=selector_scroll.set)
    controls = ttk.Frame(selector)
    selector.create_window(0, 0, window=controls, anchor='nw')
    for value, label in [('all', 'ALL DRONES')] + [(str(i), f'DRONE {i:02d}') for i in range(args.n)]:
        ttk.Radiobutton(controls, text=label, variable=selected, value=value,
                        style='Deck.TRadiobutton').pack(side='left', padx=(0, 6))
    controls.bind('<Configure>', lambda event: selector.configure(scrollregion=selector.bbox('all')))
    fleet_status = ttk.Label(root, text='Waiting for telemetry…', padding=(18, 10))
    fleet_status.pack(fill='x')
    instrument_canvas = tk.Canvas(root, height=204, background='#0b121b', highlightthickness=0)
    instrument_canvas.pack(fill='x', padx=16)
    instrument_scroll = ttk.Scrollbar(root, orient='vertical', command=instrument_canvas.yview)
    instrument_scroll.place(in_=instrument_canvas, relx=1, rely=0, relheight=1, anchor='ne')
    instrument_canvas.configure(yscrollcommand=instrument_scroll.set)
    instruments = ttk.Frame(instrument_canvas)
    instrument_window = instrument_canvas.create_window(0, 0, window=instruments, anchor='nw')
    instrument_canvas.bind('<Configure>', lambda event: instrument_canvas.itemconfigure(
        instrument_window, width=max(1, event.width - 18)))
    instruments.bind('<Configure>', lambda event: instrument_canvas.configure(
        scrollregion=instrument_canvas.bbox('all')))
    panels = {}
    def make_panel(i):
        panel = ttk.Frame(instruments, padding=(0, 5))
        flight_status = ttk.Label(panel, text=f'Drone {i}', padding=(6, 4),
                                  font=('DejaVu Sans', 11, 'bold'))
        flight_status.pack(fill='x')
        cards = ttk.Frame(panel)
        cards.pack(fill='x')
        metrics = {}
        for column, (key, label) in enumerate([
            ('altitude', 'ALTITUDE / m'), ('speed', 'GROUND / VERTICAL / m/s'),
            ('battery', 'BATTERY'), ('error', 'GOAL DISTANCE / m')
        ]):
            cards.columnconfigure(column, weight=1, uniform='metrics')
            card = ttk.Frame(cards, style='Card.TFrame', padding=12)
            card.grid(row=0, column=column, sticky='nsew', padx=3)
            ttk.Label(card, text=label, style='Card.TLabel', font=('DejaVu Sans', 9)).pack(anchor='w')
            metrics[key] = ttk.Label(card, text='—', style='Value.TLabel')
            metrics[key].pack(anchor='w', pady=(6, 0))
        return panel, metrics, flight_status
    tabs = ttk.Notebook(root)
    tabs.pack(fill='both', expand=True, padx=16, pady=(4, 16))
    path_tab = ttk.Frame(tabs)
    detail_tab = ttk.Frame(tabs)
    tabs.add(path_tab, text='  Flight path  ')
    tabs.add(detail_tab, text='  Detailed diagnostics  ')
    canvas = tk.Canvas(path_tab, background='#101c2a', highlightthickness=0)
    canvas.pack(fill='both', expand=True)
    text = tk.Text(detail_tab, font=('DejaVu Sans Mono', 11), padx=18, pady=16,
                   background='#152332', foreground='#dce8f5', relief='flat', wrap='none')
    scroll = ttk.Scrollbar(detail_tab, orient='vertical', command=text.yview)
    scroll.pack(side='right', fill='y')
    text.configure(yscrollcommand=scroll.set)
    text.pack(fill='both', expand=True)

    last_visible = None

    def refresh():
        nonlocal last_visible
        # Bound ROS work per UI frame so busy fleets cannot starve Tk events.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0)
        visible = list(range(args.n)) if selected.get() == 'all' else [int(selected.get())]
        now = time.monotonic()
        live = sum(1 for item in data if 'state' in item and now - item['state'][1] <= 2)
        fleet_status.configure(text=f'{live} / {args.n} drones reporting live position  •  {"All drones" if selected.get() == "all" else "Viewing drone " + selected.get()}')
        diagnostics = []
        targets = {}
        if visible != last_visible:
            for panel, _, _ in panels.values():
                panel.pack_forget()
            instrument_canvas.configure(height=min(420, 145 * len(visible)))
            instrument_canvas.yview_moveto(0)
        for i in visible:
            if i not in panels:
                panels[i] = make_panel(i)
            panel, metrics, flight_status = panels[i]
            if visible != last_visible:
                panel.pack(fill='x')
            for metric in metrics.values():
                metric.configure(text='—')
            flight_status.configure(text=f'Drone {i}  •  Waiting for fresh flight status', foreground='#ffbe66')
            lines = [f'DRONE {i:02d} • North / East / Down • m, m/s • attitude: degrees', '']
            def sample(key):
                item = data[i].get(key)
                if item is None:
                    lines.append(f'{key}: WAITING')
                    return None
                msg, stamp = item
                age = now - stamp
                lines.append(f'{key}: {"STALE" if age > 2 else "LIVE"} ({age:.1f}s)')
                return msg if age <= 2 else None
            state = sample('state')
            if state:
                p, v = state.pose.pose.position, state.twist.twist.linear
                metrics['altitude'].configure(text=f'{-p.z:.2f}')
                metrics['speed'].configure(text=f'{math.hypot(v.x, v.y):.2f} / {abs(v.z):.2f}')
                lines += [f'  Position N/E/D: {p.x:8.3f} {p.y:8.3f} {p.z:8.3f}',
                          f'  Velocity N/E/D: {v.x:8.3f} {v.y:8.3f} {v.z:8.3f}   speed {math.hypot(v.x,v.y,v.z):.3f}']
            control = sample('control_state')
            target = None
            if control:
                try:
                    c = json.loads(control.data)
                    target = c.get('position_target_ned_m')
                    lines += [f'  Mode: {c.get("mode")}   started: {c.get("started")}   landing: {c.get("landing")}',
                              f'  Goal NED: {target}   velocity target: {c.get("velocity_target_ned_mps")}']
                    if state and target and c.get('mode') == 'position':
                        metrics['error'].configure(text=f'{math.dist((p.x,p.y,p.z),target):.2f}')
                        lines.append(f'  Goal error: {math.dist((p.x,p.y,p.z),target):.3f} m')
                except (ValueError, TypeError):
                    lines.append('  Invalid controller telemetry')
            battery = sample('battery')
            if battery:
                remaining = f'{battery.remaining*100:.1f}%' if 0 <= battery.remaining <= 1 else 'unknown'
                metrics['battery'].configure(text=remaining)
                lines.append(f'  Battery: {remaining}   {battery.voltage_v:.2f} V   {battery.current_a:.2f} A   connected: {battery.connected}')
            attitude = sample('attitude')
            if attitude:
                w,x,y,z = attitude.q
                roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
                pitch = math.asin(max(-1., min(1.,2*(w*y-z*x))))
                yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                lines.append('  Roll/Pitch/Yaw: ' + ' / '.join(f'{math.degrees(a):.1f}' for a in (roll,pitch,yaw)))
            status = sample('vehicle_status')
            if status:
                armed = status.arming_state == 2
                offboard = status.nav_state == 14
                flight_status.configure(
                    text=f'Drone {i}  •  {"ARMED" if armed else "DISARMED"}  •  '
                         f'{"OFFBOARD" if offboard else "NAV " + str(status.nav_state)}  •  '
                         f'{"FAILSAFE" if status.failsafe else "Preflight " + ("PASS" if status.pre_flight_checks_pass else "FAIL")}',
                    foreground='#ff7885' if status.failsafe or not status.pre_flight_checks_pass else '#81e6a2')
                lines.append(f'  Armed: {status.arming_state == 2}   OFFBOARD: {status.nav_state == 14}   nav state: {status.nav_state}')
                lines.append(f'  PX4 preflight checks: {"PASS" if status.pre_flight_checks_pass else "FAIL"}   failsafe: {status.failsafe}   failure mask: {status.failure_detector_status}')
            local = sample('local_telemetry')
            if local:
                lines.append(f'  Estimator validity XY/Z: {local.xy_valid}/{local.z_valid}   velocity XY/Z: {local.v_xy_valid}/{local.v_z_valid}')
            targets[i] = target
            diagnostics.extend(lines + [''])
        last_visible = visible
        text.configure(state='normal')
        text.delete('1.0', 'end')
        text.insert('end', '\n'.join(diagnostics))
        text.configure(state='disabled')
        canvas.delete('all')
        points = [point for i in visible for point in history[i]]
        for target in targets.values():
            if target and all(math.isfinite(v) for v in target):
                points.append(tuple(target[:2]))
        points = [p for p in points if all(math.isfinite(v) for v in p)]
        width, height = canvas.winfo_width(), canvas.winfo_height()
        canvas.create_text(15, 15, anchor='nw', fill='white', text='TACTICAL MAP  /  North ↑  East →  /  trails: aircraft color · rings: goals')
        if points:
            ns, es = zip(*points)
            midn, mide = (max(ns)+min(ns))/2, (max(es)+min(es))/2
            scale = max(.01, min(max(1,width-80)/max(2,max(es)-min(es)), max(1,height-80)/max(2,max(ns)-min(ns))))
            def xy(p): return (width/2+(p[1]-mide)*scale, height/2-(p[0]-midn)*scale)
            for fraction in (.25, .5, .75):
                canvas.create_line(width*fraction, 40, width*fraction, height-20, fill='#24364b')
                canvas.create_line(20, height*fraction, width-20, height*fraction, fill='#24364b')
            canvas.create_text(15, height-18, anchor='sw', fill='#a9bfd5',
                               text=f'View width: {width/scale:.1f} m  •  Local coordinates')
            for i in visible:
                target = targets[i]
                color = colors[i % len(colors)]
                trail = [xy(p) for p in history[i] if all(math.isfinite(v) for v in p)]
                if trail:
                    x, y = trail[-1]
                    canvas.create_oval(x-5, y-5, x+5, y+5, fill=color, outline='white')
                    canvas.create_text(x+12, y-12, text=f'Drone {i}', fill=color, anchor='w')
                if len(trail)>1:
                    canvas.create_line(*[v for p in trail for v in p], fill=color, width=2)
                if target and all(math.isfinite(v) for v in target):
                    x,y=xy(target)
                    canvas.create_oval(x-6,y-6,x+6,y+6, outline=color, width=2)
        else:
            canvas.create_text(width/2, height/2, fill='#8fa6bc',
                               text='AWAITING POSITION TELEMETRY', font=('DejaVu Sans Mono', 14))
        root.after(100, refresh)
    try:
        refresh()
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
