#!/usr/bin/env python3
"""
Example script showing how to tune LQR gains for different performance characteristics.
Run this to see the effect of different Q and R matrices.
"""

import numpy as np
from main import load_config, build_model, design_lqr, simulate_response, plot_responses

def compare_lqr_tunings():
    """Compare different LQR tuning strategies."""
    
    # Load system
    cfg = load_config('config.json')
    sys_c, _ = build_model(cfg)
    
    # Define different tuning strategies
    tunings = {
        'Baseline': {
            'Q': np.eye(3),
            'R': np.array([[1]])
        },
        'Fast Response': {
            'Q': np.diag([50, 1, 0.1]),  # High angle weighting
            'R': np.array([[1]])
        },
        'Smooth Control': {
            'Q': np.diag([10, 1, 0.1]),
            'R': np.array([[10]])         # High control penalty
        },
        'High Damping': {
            'Q': np.diag([10, 10, 0.1]), # High rate weighting
            'R': np.array([[1]])
        }
    }
    
    # Simulate each tuning
    responses = {}
    initial_angle = 0.15  # ~8.6 degrees
    
    for name, params in tunings.items():
        K = design_lqr(sys_c.A, sys_c.B, params['Q'], params['R'])
        t, y, u = simulate_response(sys_c, K, T=5.0, initial_angle=initial_angle)
        responses[name] = (t, y, u)
        
        # Print gain values for reference
        print(f"{name}: K = {K.flatten()}")
    
    # Plot comparison
    plot_responses(responses)

if __name__ == '__main__':
    compare_lqr_tunings() 