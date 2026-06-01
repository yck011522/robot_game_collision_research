import math
import threading
import time
import tkinter as tk
from tkinter import ttk

from rtde_core import connect_control, connect_receive, read_actual_joints

ROBOT_IP = "192.168.0.2"

# Keep this False until you are ready to move the real robot.
RUN_MOTION = True

# servoJ stream configuration
STREAM_HZ = 125.0
LOOKAHEAD_TIME = 0.05
GAIN = 500
SERVO_SPEED = 0.5
SERVO_ACCELERATION = 0.5

# UI/rate-limit configuration
JOINT_MIN_DEG = -180.0
JOINT_MAX_DEG = 180.0
MAX_SPEED_MIN_DEG_S = 1.0
MAX_SPEED_MAX_DEG_S = 180.0
MAX_SPEED_DEFAULT_DEG_S = 30.0


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, value))


class Joint0SliderApp:
    def __init__(self) -> None:
        self.rtde_r = connect_receive(ROBOT_IP)
        current_q = read_actual_joints(self.rtde_r)

        self.base_q = current_q.copy()
        self.selected_joint_idx = 0
        current_deg = math.degrees(current_q[self.selected_joint_idx])

        self.target_deg = current_deg
        self.commanded_deg = current_deg
        self.actual_deg = current_deg
        self.max_speed_deg_s = MAX_SPEED_DEFAULT_DEG_S

        self.lock = threading.Lock()
        self.running = True

        self.rtde_c = None
        if RUN_MOTION:
            self.rtde_c = connect_control(ROBOT_IP, frequency_hz=STREAM_HZ)

        self.root = tk.Tk()
        self.root.title("UR Joint Slider Control")
        self.root.geometry("900x320")

        self.selected_joint_var = tk.StringVar(value=f"Joint {self.selected_joint_idx}")
        self.target_label_var = tk.StringVar(value=f"Target Joint {self.selected_joint_idx} (deg)")
        self.actual_label_var = tk.StringVar(value=f"Actual Joint {self.selected_joint_idx} (deg, read-only)")
        self.target_var = tk.DoubleVar(value=current_deg)
        self.actual_var = tk.DoubleVar(value=current_deg)
        self.max_speed_var = tk.DoubleVar(value=self.max_speed_deg_s)

        self.status_var = tk.StringVar(value="Connecting done. Ready.")
        self.commanded_var = tk.StringVar(value=f"Filtered command: {current_deg:.2f} deg")

        self._build_ui()

        self.worker = threading.Thread(target=self._control_loop, daemon=True)
        self.worker.start()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._refresh_ui)

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        joint_row = ttk.Frame(frame)
        joint_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(joint_row, text="Controlled joint:").pack(side=tk.LEFT)
        self.joint_selector = ttk.Combobox(
            joint_row,
            values=[f"Joint {i}" for i in range(6)],
            state="readonly",
            width=14,
            textvariable=self.selected_joint_var,
        )
        self.joint_selector.pack(side=tk.LEFT, padx=(8, 0))
        self.joint_selector.bind("<<ComboboxSelected>>", self._on_joint_selected)

        ttk.Label(frame, textvariable=self.target_label_var).pack(anchor=tk.W)
        self.target_scale = tk.Scale(
            frame,
            from_=JOINT_MIN_DEG,
            to=JOINT_MAX_DEG,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=850,
            variable=self.target_var,
            command=self._on_target_changed,
        )
        self.target_scale.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(frame, textvariable=self.actual_label_var).pack(anchor=tk.W)
        self.actual_scale = tk.Scale(
            frame,
            from_=JOINT_MIN_DEG,
            to=JOINT_MAX_DEG,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=850,
            variable=self.actual_var,
            state=tk.DISABLED,
        )
        self.actual_scale.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(frame, text="Max speed limiter (deg/s)").pack(anchor=tk.W)
        self.speed_scale = tk.Scale(
            frame,
            from_=MAX_SPEED_MIN_DEG_S,
            to=MAX_SPEED_MAX_DEG_S,
            orient=tk.HORIZONTAL,
            resolution=0.5,
            length=850,
            variable=self.max_speed_var,
            command=self._on_speed_changed,
        )
        self.speed_scale.pack(fill=tk.X, pady=(0, 12))

        ttk.Label(frame, textvariable=self.commanded_var).pack(anchor=tk.W)
        ttk.Label(frame, textvariable=self.status_var).pack(anchor=tk.W, pady=(4, 0))

        if not RUN_MOTION:
            self.status_var.set("RUN_MOTION=False. Visual/test mode only (no robot commands).")

    def _sync_selected_joint_to_robot(self) -> None:
        current_q = self.rtde_r.getActualQ()
        joint_idx = self.selected_joint_idx
        current_deg = math.degrees(current_q[joint_idx])

        with self.lock:
            self.base_q = current_q.copy()
            self.target_deg = current_deg
            self.commanded_deg = current_deg
            self.actual_deg = current_deg

        self.target_var.set(current_deg)
        self.actual_var.set(current_deg)

    def _on_joint_selected(self, _event: object) -> None:
        selected = self.selected_joint_var.get()
        new_idx = int(selected.split()[-1])
        with self.lock:
            self.selected_joint_idx = new_idx

        self.target_label_var.set(f"Target Joint {new_idx} (deg)")
        self.actual_label_var.set(f"Actual Joint {new_idx} (deg, read-only)")
        self._sync_selected_joint_to_robot()
        self.status_var.set(f"Switched to Joint {new_idx}; sliders reinitialized from robot state.")

    def _on_target_changed(self, value: str) -> None:
        with self.lock:
            self.target_deg = clamp(float(value), JOINT_MIN_DEG, JOINT_MAX_DEG)

    def _on_speed_changed(self, value: str) -> None:
        with self.lock:
            self.max_speed_deg_s = clamp(float(value), MAX_SPEED_MIN_DEG_S, MAX_SPEED_MAX_DEG_S)

    def _control_loop(self) -> None:
        dt = 1.0 / STREAM_HZ

        while self.running:
            cycle_start = None
            if self.rtde_c is not None:
                cycle_start = self.rtde_c.initPeriod()

            actual_q = self.rtde_r.getActualQ()
            with self.lock:
                joint_idx = self.selected_joint_idx
            actual_deg = math.degrees(actual_q[joint_idx])

            with self.lock:
                max_step = self.max_speed_deg_s * dt
                error = self.target_deg - self.commanded_deg
                step = clamp(error, -max_step, max_step)
                self.commanded_deg = clamp(self.commanded_deg + step, JOINT_MIN_DEG, JOINT_MAX_DEG)
                commanded_deg = self.commanded_deg
                self.actual_deg = actual_deg
                joint_idx = self.selected_joint_idx

            if self.rtde_c is not None:
                q_cmd = actual_q.copy()
                q_cmd[joint_idx] = math.radians(commanded_deg)
                self.rtde_c.servoJ(
                    q_cmd,
                    SERVO_SPEED,
                    SERVO_ACCELERATION,
                    dt,
                    LOOKAHEAD_TIME,
                    GAIN,
                )
                self.rtde_c.waitPeriod(cycle_start)
            else:
                time.sleep(dt)

        if self.rtde_c is not None:
            self.rtde_c.servoStop()
            self.rtde_c.stopScript()

    def _refresh_ui(self) -> None:
        with self.lock:
            actual = self.actual_deg
            target = self.target_deg
            commanded = self.commanded_deg
            max_speed = self.max_speed_deg_s

        self.actual_var.set(actual)
        lag = target - actual
        self.commanded_var.set(
            f"Filtered command: {commanded:.2f} deg | Actual lag: {lag:+.2f} deg | Max speed: {max_speed:.1f} deg/s"
        )

        if self.running:
            self.root.after(50, self._refresh_ui)

    def _on_close(self) -> None:
        self.running = False
        self.worker.join(timeout=2.0)
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = Joint0SliderApp()
    app.run()


if __name__ == "__main__":
    main()
