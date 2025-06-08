import json
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete, StateSpace
import matplotlib.pyplot as plt
from collections import deque


def load_config(path: str) -> dict:
    """Load JSON configuration for rocket and control parameters."""
    with open(path, 'r') as f:
        return json.load(f)


def build_model(config: dict) -> StateSpace:
    """
    Build and return the discrete-time state-space model for 1D rocket control.
    The model is sampled at the MCU loop rate (dt = 1/servo_frequency_hz).
    """
    # Physical parameters
    I = config.get('moment_of_inertia', 0.00469)
    l = config.get('tvc_to_COM', 0.147)
    T0 = config.get('constant_thrust_N', 3.5)
    k = T0 * l / I
    tau = config.get('servo_time_constant_sec', config.get('servo_delay_sec', 0.03333))

    # Continuous-time matrices (for ZOH discretization)
    A = np.array([[0, 1, 0],
                  [0, 0, k],
                  [0, 0, -1.0 / tau]])
    B = np.array([[0], [0], [1.0 / tau]])
    C = np.array([[1, 0, 0], [0, 1, 0]])
    D = np.zeros((2, 1))

    # Discretize at the MCU sample rate
    fs = config.get('servo_frequency_hz', 50)
    dt = 1.0 / fs  # sample time in seconds
    A_d, B_d, C_d, D_d, _ = cont2discrete((A, B, C, D), dt, method='zoh')
    sys_d = StateSpace(A_d, B_d, C_d, D_d, dt=dt)
    return sys_d


def design_lqr(A_d: np.ndarray, B_d: np.ndarray, Q=None, R=None) -> np.ndarray:
    """
    Design a discrete-time LQR controller.
    Solves X = A'XA - A'XB(B'XB + R)^{-1}B'XA + Q
    and returns K = (B'XB + R)^{-1} B'X A.
    """
    n, m = A_d.shape[0], B_d.shape[1]
    Q = Q if Q is not None else np.eye(n)
    R = R if R is not None else np.eye(m)
    # Solve discrete Riccati equation
    X = solve_discrete_are(A_d, B_d, Q, R)
    # Compute gain
    K = np.linalg.inv(B_d.T @ X @ B_d + R) @ (B_d.T @ X @ A_d)
    return K


def apply_servo_constraints(u_cmd: float, config: dict, dt: float, u_prev: float = None) -> float:
    """
    Enforce linkage ratio, rate & angle limits, and motor offset.
    Input u_cmd: desired TVC deflection [rad]. Output: constrained TVC [rad].
    """
    ratio = config.get('tvc_linkage_ratio', 1)
    servo_deg = np.degrees(u_cmd) * ratio

    if u_prev is not None:
        max_rate = config.get('tvc_deg_per_second_max', 125.0)
        delta_max = max_rate * dt
        prev_deg = np.degrees(u_prev) * ratio - config.get('tvc_motor_offset_x_deg', 0.0)
        servo_deg = prev_deg + np.clip(servo_deg - prev_deg, -delta_max, delta_max)

    limit = config.get('tvc_angle_range', 8.0)
    servo_deg = np.clip(servo_deg, -limit, limit)
    servo_deg += config.get('tvc_motor_offset_x_deg', 0.0)

    return np.radians(servo_deg / ratio)


class DelayBlock:
    """
    Discrete-time delay: fixed dead-time + move-time based on max servo speed.
    """
    def __init__(self, dead_time_sec: float, dt: float, speed_deg_per_sec: float):
        self.dt = dt
        self.dead_steps = int(np.round(dead_time_sec / dt))
        self.speed = speed_deg_per_sec
        self.queue = deque()
        self.step = 0
        self.last_u = 0.0
        self.last_out = 0.0

    def __call__(self, u: float) -> float:
        # compute movement delay steps
        delta_deg = abs(np.degrees(u) - np.degrees(self.last_u))
        self.last_u = u
        move_steps = int(np.round((delta_deg / self.speed) / self.dt)) if self.speed > 0 else 0
        release = self.step + self.dead_steps + move_steps
        self.queue.append((release, u))

        out = self.last_out
        if self.queue and self.queue[0][0] <= self.step:
            out = self.queue.popleft()[1]
            self.last_out = out
        self.step += 1
        return out


def simulate_response(sys_d: StateSpace, K: np.ndarray, config: dict,
                      T: float = None, initial_angle_deg: float = None,
                      controller_type: str = 'LQR') -> tuple:
    """
    Run discrete-time closed-loop sim at dt=sys_d.dt. Returns t,y,u_cmd,u_act.
    """
    dt = sys_d.dt
    fs = 1.0 / dt
    T = config.get('sim_time_seconds', 6.0) if T is None else T
    init_deg = config.get('initial_x_rot_angle', 20.0) if initial_angle_deg is None else initial_angle_deg
    x0 = np.radians(init_deg)

    n = int(T * fs)
    t = np.arange(n) * dt

    # Delay block
    delay = DelayBlock(config.get('servo_delay_sec', 0.03333), dt,
                       config.get('tvc_deg_per_second_max', 125.0))

    A, B, C, D = sys_d.A, sys_d.B, sys_d.C, sys_d.D
    x = np.zeros((n, A.shape[0])); y = np.zeros((n, C.shape[0]))
    u_cmd = np.zeros(n); u_act = np.zeros(n)
    x[0, 0] = x0
    integral = 0.0
    windup = config.get('INTEGRAL_WINDUP_LIMIT_ANGLE', 3.0)

    for k in range(n-1):
        # ctrl law
        if controller_type == 'PID':
            kp, kd, ki = (config.get(v,0.0) for v in ['STAB_KP','STAB_KD','STAB_KI'])
            e, r = x[k,0], x[k,1]
            integral = np.clip(integral + e*dt, -windup, windup)
            u = -(kp*e + kd*r + ki*integral)
        else:
            u = float(-K @ x[k])
        u_cmd[k] = u

        # apply constraints + delay
        u_c = apply_servo_constraints(u, config, dt, u_act[k-1] if k>0 else None)
        u_d = delay(u_c)
        u_act[k] = u_c

        # discrete update
        x[k+1] = A @ x[k] + B.flatten()*u_d
        y[k]   = C @ x[k] + D.flatten()*u_d

    y[-1] = C @ x[-1] + D.flatten()*u_act[-1]
    return t, y, u_cmd, u_act


def plot_responses(responses: dict, config: dict):
    """Plot attitude and actuator signals."""
    fig, (ax1, ax2) = plt.subplots(2,1, figsize=(10,6))
    for name,(t,y,_,_) in responses.items(): ax1.plot(t, np.degrees(y[:,0]), label=name)
    ax1.axhline(0, ls='--', alpha=0.6, label='Ref')
    ax1.set(xlabel='Time [s]', ylabel='Angle [deg]'); ax1.legend(); ax1.grid()

    for name,(t,_,_,u) in responses.items(): ax2.step(t, np.degrees(u), where='post', label=name)
    lim=config.get('tvc_angle_range',8.0)
    ax2.axhline(lim,ls='--',alpha=0.6); ax2.axhline(-lim,ls='--',alpha=0.6, label='±Limit')
    ax2.set(xlabel='Time [s]', ylabel='TVC [deg]'); ax2.legend(); ax2.grid()
    plt.tight_layout(); plt.show()


def main():
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument('config'); p.add_argument('--T',type=float); p.add_argument('--angle',type=float); p.add_argument('--thrust',type=float)
    args=p.parse_args()
    cfg=load_config(args.config)
    if args.thrust is not None: cfg['constant_thrust_N']=args.thrust

    sys_d=build_model(cfg)
    print(f"Using dt={sys_d.dt:.3f}s ({1/sys_d.dt:.1f}Hz)")
    K=design_lqr(sys_d.A, sys_d.B, Q=np.diag([20,0.1,1]), R=np.array([[50]]))
    init=args.angle if args.angle is not None else None
    r1 = simulate_response(sys_d, K, cfg, T=args.T, initial_angle_deg=init, controller_type='LQR')
    r2 = simulate_response(sys_d, None, cfg, T=args.T, initial_angle_deg=init, controller_type='PID')
    plot_responses({'LQR':r1,'PID':r2},cfg)
    print(f"Final LQR={np.degrees(r1[1][-1,0]):.2f}°, PID={np.degrees(r2[1][-1,0]):.2f}°")

if __name__=='__main__': main()
