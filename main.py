import json
import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import StateSpace, cont2discrete
import matplotlib.pyplot as plt
from collections import deque


def load_config(path: str) -> dict:
    """Load JSON configuration for rocket and control parameters."""
    with open(path, 'r') as f:
        return json.load(f)


class DelayBlock:
    """
    Implements a discrete-time delay combining a fixed dead-time and
    a movement-based delay using servo speed limits.
    """
    def __init__(self, dead_time_sec: float, dt: float, servo_speed_deg_per_sec: float,
                 init: float = 0.0):
        self.dt = dt
        self.dead_steps = int(round(dead_time_sec / dt))
        self.speed = servo_speed_deg_per_sec
        self.queue = deque()
        self.current_step = 0
        self.last_u = init
        self.last_out = init

    def __call__(self, u: float) -> float:
        # Compute movement-based additional delay (in steps)
        delta_deg = abs(np.degrees(u) - np.degrees(self.last_u))
        self.last_u = u
        # time for movement at max servo speed
        move_time = delta_deg / self.speed if self.speed > 0 else 0.0
        move_steps = int(round(move_time / self.dt))
        # schedule release for this command
        release_step = self.current_step + self.dead_steps + move_steps
        self.queue.append((release_step, u))

        # default output until due
        out = self.last_out
        # release queued commands when scheduled
        while self.queue and self.queue[0][0] <= self.current_step:
            out = self.queue.popleft()[1]
        self.last_out = out
        self.current_step += 1
        return out


def build_model(config: dict) -> tuple:
    I = config.get('moment_of_inertia', 0.00469)
    l = config.get('tvc_to_COM', 0.147)
    T0 = config.get('constant_thrust_N', 3.5)
    k = T0 * l / I
    tau = config.get('servo_time_constant_sec', config.get('servo_delay_sec', 0.03333))

    A = np.array([[0, 1, 0],
                  [0, 0, k],
                  [0, 0, -1.0 / tau]])
    B = np.array([[0], [0], [1.0 / tau]])
    C = np.array([[1, 0, 0], [0, 1, 0]])
    D = np.zeros((2, 1))

    sys_c = StateSpace(A, B, C, D)
    fs = config.get('servo_frequency_hz', 50)
    dt = 1.0 / fs
    A_d, B_d, C_d, D_d, _ = cont2discrete((A, B, C, D), dt, method='zoh')
    sys_d = StateSpace(A_d, B_d, C_d, D_d, dt=dt)

    return sys_c, sys_d


def design_lqr(A: np.ndarray, B: np.ndarray, Q=None, R=None) -> np.ndarray:
    n = A.shape[0]
    m = B.shape[1]
    Q = Q if Q is not None else np.eye(n)
    R = R if R is not None else np.eye(m)
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def apply_servo_constraints(u_cmd: float, config: dict, dt: float, u_prev: float = None) -> float:
    ratio = config.get('tvc_linkage_ratio', 1)
    servo_cmd_deg = np.degrees(u_cmd) * ratio

    if u_prev is not None:
        max_rate = config.get('tvc_deg_per_second_max', 125.0)
        max_delta = max_rate * dt
        prev_servo_deg = np.degrees(u_prev) * ratio
        prev_servo_deg -= config.get('tvc_motor_offset_x_deg', 0.0)
        delta = servo_cmd_deg - prev_servo_deg
        servo_cmd_deg = prev_servo_deg + np.clip(delta, -max_delta, max_delta)

    max_tvc = config.get('tvc_angle_range', 8.0)
    servo_cmd_deg = np.clip(servo_cmd_deg, -max_tvc, max_tvc)
    servo_cmd_deg += config.get('tvc_motor_offset_x_deg', 0.0)

    return np.radians(servo_cmd_deg / ratio)


def simulate_response(sys, K, config, T=None, initial_angle=None, controller_type='LQR'):
    fs = config.get('servo_frequency_hz', 50)
    dt = 1.0 / fs
    T = config.get('sim_time_seconds', 6.0) if T is None else T
    initial_angle = np.radians(config.get('initial_x_rot_angle', 20.0)) if initial_angle is None else initial_angle
    n_steps = int(T * fs)
    t = np.linspace(0, T, n_steps)

    delay = DelayBlock(config.get('servo_delay_sec', 0.03333), dt,
                       config.get('tvc_deg_per_second_max', 125.0), init=0.0)

    A, B, C = sys.A, sys.B, sys.C
    x = np.zeros((n_steps, A.shape[0]))
    y = np.zeros((n_steps, C.shape[0]))
    u_cmd = np.zeros(n_steps)
    u_act = np.zeros(n_steps)

    x[0, 0] = initial_angle
    integral_err = 0.0
    windup_limit = config.get('INTEGRAL_WINDUP_LIMIT_ANGLE', 3.0)

    for i in range(n_steps - 1):
        if controller_type == 'PID':
            kp, kd, ki = config.get('STAB_KP', 0.0), config.get('STAB_KD', 0.0), config.get('STAB_KI', 0.0)
            err, rate = x[i, 0], x[i, 1]
            integral_err = np.clip(integral_err + err * dt, -windup_limit, windup_limit)
            u = -(kp * err + kd * rate + ki * integral_err)
        else:
            u = (-K @ x[i]).item()
        u_cmd[i] = u

        u_prev = u_act[i-1] if i > 0 else None
        u_cons = apply_servo_constraints(u, config, dt, u_prev)
        u_delayed = delay(u_cons)
        u_act[i] = u_cons

        x_dot = A @ x[i] + B.flatten() * u_delayed
        x[i+1] = x[i] + dt * x_dot
        y[i] = C @ x[i]

    y[-1] = C @ x[-1]
    return t, y, u_cmd, u_act


def plot_responses(responses: dict, config: dict):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    for name, (t, y, _, _) in responses.items():
        ax1.plot(t, np.degrees(y[:, 0]), label=name)
    ax1.axhline(0, linestyle='--', alpha=0.5, label='Target')
    ax1.set(xlabel='Time (s)', ylabel='Angle (deg)', title='Attitude Control')
    ax1.legend()
    ax1.grid(alpha=0.3)

    for name, (t, _, _, u_act) in responses.items():
        ax2.plot(t, np.degrees(u_act), label=name)
    lim = config.get('tvc_angle_range', 8.0)
    ax2.axhline(lim, linestyle='--', alpha=0.5, label='TVC Limits')
    ax2.axhline(-lim, linestyle='--', alpha=0.5)
    ax2.set(xlabel='Time (s)', ylabel='TVC Deflection (deg)', title='Control Signals')
    ax2.legend()
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser("LQR vs PID Rocket Control Simulation")
    parser.add_argument('config', help='JSON configuration file')
    parser.add_argument('--T', type=float, default=None, help='Simulation time (s)')
    parser.add_argument('--angle', type=float, default=None, help='Initial angle (deg)')
    parser.add_argument('--thrust', type=float, default=None, help='Constant thrust (N)')
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.thrust is not None:
        cfg['constant_thrust_N'] = args.thrust

    sys_c, sys_d = build_model(cfg)
    Q = np.diag([20, 0.1, 1])
    R = np.array([[50]])
    K = design_lqr(sys_c.A, sys_c.B, Q, R)

    init_rad = np.radians(args.angle) if args.angle is not None else None
    resp_lqr = simulate_response(sys_c, K, cfg, T=args.T, initial_angle=init_rad, controller_type='LQR')
    resp_pid = simulate_response(sys_c, None, cfg, T=args.T, initial_angle=init_rad, controller_type='PID')

    plot_responses({'LQR': resp_lqr, 'PID': resp_pid}, cfg)

    print(f"Final angles => LQR: {np.degrees(resp_lqr[1][-1,0]):.2f}°, PID: {np.degrees(resp_pid[1][-1,0]):.2f}°")


if __name__ == '__main__':
    main()
