#!/usr/bin/env python3
"""
Example script showing how to tune LQR gains for different performance characteristics.
Run this to see the effect of different Q and R matrices.
"""

import numpy as np
import matplotlib.pyplot as plt
from main import load_config, build_model, design_lqr, simulate_response

def compare_lqr_tunings():
    """Compare different LQR tuning strategies."""
    
    # Load system
    cfg = load_config('config.json')
    sys_c, _ = build_model(cfg)
    
    # Define different tuning strategies
    tunings = {
        'Baseline': {
            'Q': np.eye(3),
            'R': np.array([[1]]),
            'description': 'Equal state weighting, standard control penalty'
        },
        'Fast Response': {
            'Q': np.diag([50, 1, 0.1]),  # High angle weighting
            'R': np.array([[1]]),
            'description': 'High angle penalty for fast settling'
        },
        'Smooth Control': {
            'Q': np.diag([10, 1, 0.1]),
            'R': np.array([[10]]),         # High control penalty
            'description': 'High control penalty for smooth actuation'
        },
        'High Damping': {
            'Q': np.diag([10, 10, 0.1]), # High rate weighting
            'R': np.array([[1]]),
            'description': 'High rate penalty for better damping'
        },
        'Aggressive': {
            'Q': np.diag([100, 5, 0.1]),
            'R': np.array([[0.1]]),        # Low control penalty
            'description': 'Very aggressive response'
        }
    }
    
    # Simulate each tuning
    responses = {}
    initial_angle = np.radians(15)  # 15 degrees initial disturbance
    sim_time = 5.0
    
    print("=== LQR Tuning Comparison ===")
    print(f"Initial angle: {np.degrees(initial_angle):.1f}°")
    print(f"Simulation time: {sim_time}s")
    print()
    
    for name, params in tunings.items():
        K = design_lqr(sys_c.A, sys_c.B, params['Q'], params['R'])
        t, y, u_cmd, u_actual = simulate_response(
            sys_c, K, cfg, T=sim_time, initial_angle=initial_angle, controller_type='LQR')
        responses[name] = (t, y, u_cmd, u_actual)
        
        # Calculate performance metrics
        final_angle = np.degrees(y[-1, 0])
        settling_time = calculate_settling_time(t, y[:, 0], tolerance=0.02)  # 2% settling
        max_control = np.max(np.abs(u_actual))
        
        # Print results
        print(f"{name}:")
        print(f"  Description: {params['description']}")
        print(f"  Gains: K = [{K[0,0]:.2f}, {K[0,1]:.2f}, {K[0,2]:.2f}]")
        print(f"  Final angle: {final_angle:.2f}°")
        print(f"  Settling time: {settling_time:.2f}s")
        print(f"  Max control: {np.degrees(max_control):.1f}°")
        print()
    
    # Plot comparison
    plot_tuning_comparison(responses, cfg)

def calculate_settling_time(t, angle, tolerance=0.02):
    """Calculate 2% settling time."""
    final_value = angle[-1]
    tolerance_band = abs(final_value) + tolerance
    
    # Find last time outside tolerance band
    outside_tolerance = np.abs(angle) > tolerance_band
    if not np.any(outside_tolerance):
        return 0.0
    
    last_outside_idx = np.where(outside_tolerance)[0][-1]
    return t[last_outside_idx] if last_outside_idx < len(t)-1 else t[-1]

def plot_tuning_comparison(responses, config):
    """Plot comparison of different LQR tunings."""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Angle responses
    for name, (t, y, u_cmd, u_actual) in responses.items():
        angle = np.degrees(y[:, 0])
        ax1.plot(t, angle, label=name, linewidth=2)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (degrees)')
    ax1.set_title('Angle Response Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    # Plot 2: Control signals
    for name, (t, y, u_cmd, u_actual) in responses.items():
        control = np.degrees(u_actual)
        ax2.plot(t, control, label=name, linewidth=2)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('TVC Deflection (degrees)')
    ax2.set_title('Control Signal Comparison')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Add TVC limits
    max_angle = config.get('tvc_angle_range', 8.0)
    ax2.axhline(y=max_angle, color='r', linestyle='--', alpha=0.5, label='TVC Limit')
    ax2.axhline(y=-max_angle, color='r', linestyle='--', alpha=0.5)
    
    # Plot 3: Angular velocity
    for name, (t, y, u_cmd, u_actual) in responses.items():
        rate = np.degrees(y[:, 1])
        ax3.plot(t, rate, label=name, linewidth=2)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angular Velocity (deg/s)')
    ax3.set_title('Angular Velocity Comparison')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Control effort (cumulative)
    for name, (t, y, u_cmd, u_actual) in responses.items():
        control_effort = np.cumsum(np.abs(u_actual)) * (t[1] - t[0])
        ax4.plot(t, control_effort, label=name, linewidth=2)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Cumulative Control Effort')
    ax4.set_title('Control Effort Comparison')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def interactive_tuning():
    """Interactive tuning session - modify Q and R values."""
    cfg = load_config('config.json')
    sys_c, _ = build_model(cfg)
    
    print("=== Interactive LQR Tuning ===")
    print("Modify Q and R matrices to see the effect on performance")
    print("Current system: 3 states [angle, rate, TVC_deflection]")
    print()
    
    while True:
        try:
            print("Enter Q matrix diagonal values (angle, rate, TVC):")
            q_vals = input("Q diagonal [10, 1, 0.1]: ").strip()
            if q_vals:
                q_list = [float(x.strip()) for x in q_vals.split(',')]
                Q = np.diag(q_list)
            else:
                Q = np.diag([10, 1, 0.1])
            
            print("Enter R value (control penalty):")
            r_val = input("R [1]: ").strip()
            R = np.array([[float(r_val) if r_val else 1]])
            
            # Design and simulate
            K = design_lqr(sys_c.A, sys_c.B, Q, R)
            t, y, u_cmd, u_actual = simulate_response(
                sys_c, K, cfg, T=5.0, initial_angle=np.radians(15), controller_type='LQR')
            
            # Show results
            final_angle = np.degrees(y[-1, 0])
            settling_time = calculate_settling_time(t, y[:, 0])
            max_control = np.degrees(np.max(np.abs(u_actual)))
            
            print(f"\nResults:")
            print(f"  Gains: K = [{K[0,0]:.2f}, {K[0,1]:.2f}, {K[0,2]:.2f}]")
            print(f"  Final angle: {final_angle:.2f}°")
            print(f"  Settling time: {settling_time:.2f}s")
            print(f"  Max control: {max_control:.1f}°")
            
            cont = input("\nContinue tuning? (y/n): ").strip().lower()
            if cont != 'y':
                break
                
        except (ValueError, KeyboardInterrupt):
            print("Exiting...")
            break

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--interactive':
        interactive_tuning()
    else:
        compare_lqr_tunings() 