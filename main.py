import json
import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are
from scipy.signal import StateSpace, cont2discrete
import matplotlib.pyplot as plt


def load_config(path: str) -> dict:
    """Load JSON configuration for rocket and control parameters."""
    with open(path, 'r') as f:
        return json.load(f)


def build_model(config: dict) -> tuple:
    """
    Build continuous and discrete state-space models for 1D rocket control.
    Returns: (sys_c, sys_d) - continuous and discrete StateSpace systems
    """
    # Linearized rocket dynamics: [θ, ω, δ]
    I = config.get('moment_of_inertia', 0.00469)
    l = config.get('tvc_to_COM', 0.147)
    T0 = config.get('constant_thrust_N', 3.5)
    k = T0 * l / I
    tau = config.get('servo_delay_sec', 0.03333)

    A = np.array([[0, 1, 0],
                  [0, 0, k],
                  [0, 0, -1/tau]])
    B = np.array([[0], [0], [1/tau]])
    C = np.array([[1, 0, 0], [0, 1, 0]])
    D = np.zeros((2, 1))

    sys_c = StateSpace(A, B, C, D)

    # Discretize with ZOH at servo rate
    fs = config.get('servo_frequency_hz', 50)
    Ts = 1.0 / fs
    A_d, B_d, C_d, D_d, _ = cont2discrete((A, B, C, D), Ts, method='zoh')
    sys_d = StateSpace(A_d, B_d, C_d, D_d, dt=Ts)
    return sys_c, sys_d


def design_lqr(A: np.ndarray, B: np.ndarray, Q=None, R=None) -> np.ndarray:
    """Design continuous-time LQR controller."""
    n, m = A.shape[0], B.shape[1]
    Q = Q if Q is not None else np.eye(n)
    R = R if R is not None else np.eye(m)

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


def design_dlqr(A_d: np.ndarray, B_d: np.ndarray, Q=None, R=None) -> np.ndarray:
    """Design discrete-time LQR by solving discrete ARE."""
    n, m = A_d.shape[0], B_d.shape[1]
    Q = Q if Q is not None else np.eye(n)
    R = R if R is not None else np.eye(m)

    P_d = solve_discrete_are(A_d, B_d, Q, R)
    K_d = np.linalg.solve(R + B_d.T @ P_d @ B_d, B_d.T @ P_d @ A_d)
    return K_d


def apply_servo_constraints(u_cmd, config, dt, u_prev=None):
    """
    Apply servo rate and angle limits with linkage ratio.
    Returns constrained servo command in radians.
    """
    tvc_angle_deg = np.degrees(u_cmd)
    if u_prev is not None:
        max_rate = config.get('tvc_deg_per_second_max', 125.0)
        max_change = max_rate * dt
        tvc_prev_deg = np.degrees(u_prev)
        motor_offset = config.get('tvc_motor_offset_x_deg', 0.0)
        tvc_prev_deg -= motor_offset
        du = tvc_angle_deg - tvc_prev_deg
        tvc_angle_deg = tvc_prev_deg + np.clip(du, -max_change, max_change)
    max_tvc_angle = config.get('tvc_angle_range', 8.0)
    tvc_angle_deg = np.clip(tvc_angle_deg, -max_tvc_angle, max_tvc_angle)
    motor_offset = config.get('tvc_motor_offset_x_deg', 0.0)
    tvc_angle_deg += motor_offset
    return np.radians(tvc_angle_deg)


def simulate_response(sys_d: StateSpace, K, config: dict,
                      T=None, initial_angle=None,
                      controller_type='LQR') -> tuple:
    """
    Simulate closed-loop response using the discrete model.
    Returns: (t, y, u_cmd, u_actual)
    """
    fs = config.get('servo_frequency_hz', 50)
    Ts = 1.0 / fs
    if T is None:
        T = config.get('sim_time_seconds', 6.0)
    npts = int(T * fs)
    t = np.linspace(0, T, npts)

    A_d, B_d, C_d = sys_d.A, sys_d.B, sys_d.C
    x = np.zeros((npts, A_d.shape[0]))
    y = np.zeros((npts, C_d.shape[0]))
    u_cmd = np.zeros(npts)
    u_actual = np.zeros(npts)

    # initial state
    if initial_angle is None:
        initial_angle = np.radians(config.get('initial_x_rot_angle', 20.0))
    x[0, 0] = initial_angle

    for k in range(npts - 1):
        if controller_type == 'PID':
            kp = config.get('STAB_KP', 0.0)
            ki = config.get('STAB_KI', 0.0)
            kd = config.get('STAB_KD', 0.0)
            # PID on angle and rate
            angle = x[k, 0]
            rate = x[k, 1]
            # simple PI-D using state
            u_val = -(kp * angle + kd * rate)  # no LPF on D per request
            # integrate error for I
            # approximate integral: sum of angle*Ts
            if k == 0:
                integral = angle * Ts
            else:
                integral += angle * Ts
                integral = np.clip(integral,
                                   -config.get('INTEGRAL_WINDUP_LIMIT_ANGLE', 3.0),
                                   config.get('INTEGRAL_WINDUP_LIMIT_ANGLE', 3.0))
            u_val -= ki * integral
        else:
            u_val = -K @ x[k, :]
            u_val = float(u_val)

        u_cmd[k] = u_val
        u_prev = u_actual[k-1] if k > 0 else None
        u_act = apply_servo_constraints(u_cmd[k], config, Ts, u_prev)
        u_actual[k] = u_act

        # discrete update
        x[k+1, :] = A_d @ x[k, :] + (B_d.flatten() * u_actual[k])
        y[k, :] = C_d @ x[k, :]

    # last output
    y[-1, :] = C_d @ x[-1, :]
    u_cmd[-1] = u_cmd[-2]
    u_actual[-1] = apply_servo_constraints(u_cmd[-1], config, Ts, u_actual[-2] if npts>1 else None)

    return t, y, u_cmd, u_actual


def plot_responses(responses: dict, config: dict):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    for name, (t, y, _, u_act) in responses.items():
        ax1.plot(t, np.degrees(y[:, 0]), label=f"{name}: θ")
        ax2.plot(t, np.degrees(u_act), label=f"{name}: TVC")
    ax1.set(title='Angle Response', xlabel='Time (s)', ylabel='Angle (deg)')
    ax2.set(title='TVC Command', xlabel='Time (s)', ylabel='Deflection (deg)')
    for ax in (ax1, ax2):
        ax.grid(True)
        ax.legend()
    plt.tight_layout()
    plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('config')
    parser.add_argument('--T', type=float)
    parser.add_argument('--angle', type=float)
    parser.add_argument('--thrust', type=float, default=3.5)
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg['constant_thrust_N'] = args.thrust
    sys_c, sys_d = build_model(cfg)

    # continuous LQR
    Q = np.diag([20, 0.1, 1])
    R = np.array([[50]])
    Kc = design_lqr(sys_c.A, sys_c.B, Q, R)

    # discrete LQR
    Kd = design_dlqr(sys_d.A, sys_d.B, Q, R)

    # initial angle override
    init = np.radians(args.angle) if args.angle else None

    resp_lqr_c = simulate_response(sys_d, Kc, cfg, T=args.T, initial_angle=init, controller_type='LQR')
    resp_lqr_d = simulate_response(sys_d, Kd, cfg, T=args.T, initial_angle=init, controller_type='LQR')
    resp_pid   = simulate_response(sys_d, None, cfg, T=args.T, initial_angle=init, controller_type='PID')

    plot_responses({'LQR_cont→ZOH': resp_lqr_c,
                    'LQR_discrete': resp_lqr_d,
                    'PID': resp_pid},
                   cfg)

    print(f"Final angles (deg): contLQR {np.degrees(resp_lqr_c[1][-1,0]):.2f}, "
          f"discLQR {np.degrees(resp_lqr_d[1][-1,0]):.2f}, "
          f"PID {np.degrees(resp_pid[1][-1,0]):.2f}")

if __name__ == '__main__':
    main()
