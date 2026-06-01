import math

from rtde_core import connect_control, connect_receive, read_actual_joints, stream_servoj

ROBOT_IP = "192.168.0.2"

# Keep this False for dry run / read-only checks.
RUN_MOTION = True

# Streaming configuration
STREAM_HZ = 50
DURATION_S = 12.0

# Motion shape on joint 0
AMPLITUDE_RAD = 0.20
WAVE_HZ = 0.25


def sinusoid_trajectory(
    base_q: list[float],
    duration_s: float,
    stream_hz: float,
    amplitude_rad: float,
    wave_hz: float,
):
    total_steps = max(1, int(duration_s * stream_hz))
    for i in range(total_steps):
        t = i / stream_hz
        q = base_q.copy()
        q[0] = base_q[0] + amplitude_rad * math.sin(2.0 * math.pi * wave_hz * t)
        yield q


def main() -> None:
    rtde_r = connect_receive(ROBOT_IP)
    current_q = read_actual_joints(rtde_r)

    if not RUN_MOTION:
        print("RUN_MOTION is False. Read-only mode, no servoJ command sent.")
        return

    rtde_c = connect_control(ROBOT_IP, frequency_hz=STREAM_HZ)
    try:
        print(
            f"Starting servoJ stream: {STREAM_HZ:.1f} Hz for {DURATION_S:.2f}s "
            f"({int(DURATION_S * STREAM_HZ)} setpoints planned)"
        )
        traj = sinusoid_trajectory(
            base_q=current_q,
            duration_s=DURATION_S,
            stream_hz=STREAM_HZ,
            amplitude_rad=AMPLITUDE_RAD,
            wave_hz=WAVE_HZ,
        )
        stream_servoj(rtde_c, traj, frequency_hz=STREAM_HZ, lookahead_time=0.03, gain=500)
    finally:
        rtde_c.servoStop()
        rtde_c.stopScript()


if __name__ == "__main__":
    main()
