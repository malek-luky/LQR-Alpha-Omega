import json
import numpy as np
from scipy.linalg import solve_continuous_are
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
    T0 = config.get('constant_thrust_N', 3.5)  # Default to 3.5N as specified
    k = T0 * l / I  # Control effectiveness
    tau = config.get('servo_delay_sec', 0.03333)  # Actuator time constant
    
    A = np.array([[0, 1, 0], [0, 0, k], [0, 0, -1/tau]])
    B = np.array([[0], [0], [1/tau]])
    C = np.array([[1, 0, 0], [0, 1, 0]])  # Measure angle and rate
    D = np.zeros((2, 1))

    sys_c = StateSpace(A, B, C, D)
    
    # Discretize for digital implementation
    fs = config.get('servo_frequency_hz', 50)
    A_d, B_d, C_d, D_d, _ = cont2discrete((A, B, C, D), 1/fs, method='zoh')
    sys_d = StateSpace(A_d, B_d, C_d, D_d, dt=1/fs)
    return sys_c, sys_d


def design_lqr(A: np.ndarray, B: np.ndarray, Q=None, R=None) -> np.ndarray:
    """Design LQR controller using Algebraic Riccati Equation."""
    n, m = A.shape[0], B.shape[1]
    Q = Q if Q is not None else np.eye(n)
    R = R if R is not None else np.eye(m)
    
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


def apply_servo_constraints(u_cmd, config, dt, u_prev=None):
    """
    Apply servo rate and angle limits with linkage ratio.
    Returns constrained servo command in radians.
    """
    # Convert control command to TVC angle (radians to degrees)
    tvc_angle_deg = np.degrees(u_cmd)
    
    # Apply rate limiting if previous command is available
    if u_prev is not None:
        max_rate = config.get('tvc_deg_per_second_max', 125.0)
        max_change = max_rate * dt
        tvc_prev_deg = np.degrees(u_prev)
        
        # Remove motor offset from previous command for rate calculation
        motor_offset = config.get('tvc_motor_offset_x_deg', 0.0)
        tvc_prev_deg -= motor_offset
        
        # Apply rate limit
        du = tvc_angle_deg - tvc_prev_deg
        du_limited = np.clip(du, -max_change, max_change)
        tvc_angle_deg = tvc_prev_deg + du_limited
    
    # Apply TVC angle range limit
    max_tvc_angle = config.get('tvc_angle_range', 8.0)
    tvc_angle_deg = np.clip(tvc_angle_deg, -max_tvc_angle, max_tvc_angle)
    
    # Add motor offset (calibration error)
    motor_offset = config.get('tvc_motor_offset_x_deg', 0.0)
    tvc_angle_deg += motor_offset
    
    return np.radians(tvc_angle_deg)  # Return in radians for simulation


def simulate_response(sys, K, config, T=None, initial_angle=None, controller_type='LQR') -> tuple:
    """
    Simulate closed-loop response with realistic servo constraints.
    Returns: (t, y, u, u_actual) - time, output, commanded control, and actual control
    """
    # Use config defaults if not specified
    if T is None:
        T = config.get('sim_time_seconds', 6.0)
    if initial_angle is None:
        initial_angle = np.radians(config.get('initial_x_rot_angle', 20.0))
    
    # Simulation at servo frequency
    fs = config.get('servo_frequency_hz', 50)
    dt = 1.0 / fs
    npts = int(T * fs)
    t = np.linspace(0, T, npts)
    
    # System matrices
    A, B, C = sys.A, sys.B, sys.C
    
    # Initialize state and output arrays
    n_states, n_outputs = A.shape[0], C.shape[0]
    x = np.zeros((npts, n_states))
    y = np.zeros((npts, n_outputs))
    u_cmd = np.zeros(npts)  # Commanded control
    u_actual = np.zeros(npts)  # Actual control after constraints
    
    # PID-specific variables
    integral_error = 0.0
    integral_limit = config.get('INTEGRAL_WINDUP_LIMIT_ANGLE', 3.0)
    
    # Initial condition
    x[0, 0] = initial_angle  # Initial angle
    
    # Simulation loop with servo constraints
    for i in range(npts - 1):
        # Calculate control command based on controller type
        if controller_type == 'PID':
            # PID Controller implementation
            kp = config.get('STAB_KP', 0.0)
            kd = config.get('STAB_KD', 0.0)
            ki = config.get('STAB_KI', 0.0)
            
            # Current angle and angular velocity
            angle_error = x[i, 0]  # Error from 0 (target angle)
            angular_velocity = x[i, 1]
            
            # Update integral with windup protection
            integral_error += angle_error * dt
            integral_error = np.clip(integral_error, -integral_limit, integral_limit)
            
            # PID control law
            u_val = -(kp * angle_error + kd * angular_velocity + ki * integral_error)
        else:
            # LQR Controller (state feedback)
            u_val = -K @ x[i, :]
            u_val = u_val[0] if u_val.ndim > 0 else u_val
        
        u_cmd[i] = u_val
        
        # Apply servo constraints and motor offset
        u_prev = u_actual[i-1] if i > 0 else None
        u_constrained = apply_servo_constraints(u_cmd[i], config, dt, u_prev)
        u_actual[i] = u_constrained
        
        # Update state with actual control input
        x_dot = A @ x[i, :] + B.flatten() * u_actual[i]
        x[i+1, :] = x[i, :] + dt * x_dot
        y[i, :] = C @ x[i, :]
    
    # Final values
    if controller_type == 'PID':
        kp = config.get('STAB_KP', 0.0)
        kd = config.get('STAB_KD', 0.0)
        ki = config.get('STAB_KI', 0.0)
        angle_error = x[-1, 0]
        angular_velocity = x[-1, 1]
        integral_error += angle_error * dt
        integral_error = np.clip(integral_error, -integral_limit, integral_limit)
        u_val = -(kp * angle_error + kd * angular_velocity + ki * integral_error)
    else:
        u_val = -K @ x[-1, :]
        u_val = u_val[0] if u_val.ndim > 0 else u_val
    
    u_cmd[-1] = u_val
    u_prev = u_actual[-2] if len(u_actual) > 1 else None
    u_actual[-1] = apply_servo_constraints(u_cmd[-1], config, dt, u_prev)
    y[-1, :] = C @ x[-1, :]
    
    return t, y, u_cmd, u_actual


def plot_responses(responses: dict, config: dict):
    """Plot angle responses and control signals for comparison."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot angle responses
    for name, (t, y, u_cmd, u_actual) in responses.items():
        angle = y[:, 0] if y.ndim > 1 else y
        ax1.plot(t, np.degrees(angle), label=f"{name}: θ", linewidth=2)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (degrees)')
    ax1.set_title('Rocket Attitude Control: LQR vs PID')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Add reference line at 0 degrees
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5, label='Target')
    
    # Plot control signals
    for name, (t, y, u_cmd, u_actual) in responses.items():
        ax2.plot(t, np.degrees(u_actual), label=f"{name}: TVC angle", linewidth=2)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('TVC Deflection (degrees)')
    ax2.set_title('Control Signal Comparison')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Add TVC angle limits
    max_angle = config.get('tvc_angle_range', 8.0)
    ax2.axhline(y=max_angle, color='r', linestyle='--', alpha=0.5, label='TVC Limit')
    ax2.axhline(y=-max_angle, color='r', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser("LQR vs PID Rocket Control Simulation")
    parser.add_argument('config', help='JSON configuration file')
    parser.add_argument('--T', type=float, default=None, 
                       help='Simulation time (seconds) - overrides config')
    parser.add_argument('--angle', type=float, default=None, 
                       help='Initial angle disturbance (degrees) - overrides config')
    parser.add_argument('--thrust', type=float, default=3.5,
                       help='Constant thrust (N)')
    args = parser.parse_args()

    # Load configuration and build model
    cfg = load_config(args.config)
    
    # Override thrust if specified
    cfg['constant_thrust_N'] = args.thrust
    
    # Build system model
    sys_c, sys_d = build_model(cfg)

    # Design LQR controller
    K_lqr = design_lqr(sys_c.A, sys_c.B)

    # Convert angle argument to radians if provided
    initial_angle_rad = None
    if args.angle is not None:
        initial_angle_rad = np.radians(args.angle)

    # Simulate and compare responses
    t, y_lqr, u_cmd_lqr, u_actual_lqr = simulate_response(
        sys_c, K_lqr, cfg, T=args.T, initial_angle=initial_angle_rad, controller_type='LQR')
    t, y_pid, u_cmd_pid, u_actual_pid = simulate_response(
        sys_c, None, cfg, T=args.T, initial_angle=initial_angle_rad, controller_type='PID')

    # Plot comparison
    plot_responses({
        'LQR': (t, y_lqr, u_cmd_lqr, u_actual_lqr),
        'PID': (t, y_pid, u_cmd_pid, u_actual_pid)
    }, cfg)

    # Print final values
    print(f"Final angle - LQR: {np.degrees(y_lqr[-1, 0]):.2f}°, PID: {np.degrees(y_pid[-1, 0]):.2f}°")


if __name__ == '__main__':
    main()
