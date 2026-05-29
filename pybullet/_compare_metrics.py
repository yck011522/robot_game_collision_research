import json

rows = []
for hz in (30, 15, 10, 5):
    with open(f"pybullet/_metrics_gui{hz}.json") as f:
        rows.append((hz, json.load(f)))

print(
    f"{'hz':>4} {'ctrl_fps':>10} {'gui_fps':>9} {'ctrl_dt_p50':>13} {'ctrl_dt_p99':>13} {'gui_dt_p50':>11} {'input_lat_p50':>14} {'resize_p99':>12}"
)
for hz, r in rows:
    print(
        f"{hz:>4} "
        f"{r['ctrl_fps']:>10.1f} "
        f"{r['gui_fps']:>9.1f} "
        f"{r['ctrl_dt_ms']['p50']:>13.1f} "
        f"{r['ctrl_dt_ms']['p99']:>13.1f} "
        f"{r['gui_dt_ms']['p50']:>11.1f} "
        f"{(r['input_latency_ms']['p50'] if r['input_latency_ms'] else 0):>14.1f} "
        f"{(r['resize_stall_ms']['p99'] if r['resize_stall_ms'] else 0):>12.1f}"
    )
