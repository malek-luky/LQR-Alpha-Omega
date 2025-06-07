# LQR Rocket Control System

A Linear Quadratic Regulator (LQR) implementation for rocket attitude control, with comparison capabilities against PID controllers.

## Overview

This system provides:
- **LQR Controller Design**: Optimal state-feedback control for rocket stabilization
- **PID Comparison**: Side-by-side comparison with traditional PID control
- **1D/2D Models**: Support for single-axis or dual-axis control
- **Configurable Parameters**: JSON-based configuration for easy tuning
- **Visualization**: Matplotlib-based response plotting

## Installation

### Prerequisites
```bash
pip install numpy scipy matplotlib
```

### Dependencies
- `numpy`: Numerical computations
- `scipy`: Control systems and linear algebra
- `matplotlib`: Plotting and visualization

## Usage

### Basic Command
```bash
python main.py path/to/config.json --model 1d --T 6
```

### Command Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `config` | str | Required | Path to JSON configuration file |
| `--model` | str | `1d` | Control model: `1d` (single-axis) or `2d` (dual-axis) |
| `--T` | float | `5.0` | Simulation time in seconds |

### Examples

**Single-axis control for 6 seconds:**
```bash
python main.py my_rocket.json --model 1d --T 6
```

**Dual-axis control for 10 seconds:**
```bash
python main.py my_rocket.json --model 2d --T 10
```

**Quick 3-second test:**
```bash
python main.py my_rocket.json --T 3
```

## Configuration

The system uses JSON configuration files to define rocket parameters and control settings.

### Key Configuration Parameters

#### Physical Parameters
```json
{
    "mass": 0.292,                    // Rocket mass (kg)
    "moment_of_inertia": 0.00469,     // Moment of inertia (kg⋅m²)
    "tvc_to_COM": 0.147,              // Distance from TVC to center of mass (m)
    "constant_thrust_N": 6            // Thrust force (N)
}
```

#### Control System Parameters
```json
{
    "servo_frequency_hz": 50,         // Control loop frequency (Hz)
    "servo_delay_sec": 0.03333,       // Actuator delay (seconds)
    "tvc_angle_range": 8.0,           // Max TVC deflection (degrees)
    "tvc_deg_per_second_max": 125.0   // Max TVC rate (deg/s)
}
```

#### PID Parameters (for comparison)
```json
{
    "STAB_KP": 0.15,                  // Proportional gain
    "STAB_KI": 0.08,                  // Integral gain  
    "STAB_KD": 0.055                  // Derivative gain
}
```

### Sample Configuration
See `my_rocket.json` for a complete example configuration.

## System Models

### 1D Model (Single Axis)
- **States**: `[θ, ω, δ]` (angle, angular velocity, TVC deflection)
- **Input**: TVC command
- **Outputs**: Angle and angular velocity

### 2D Model (Dual Axis)
- **States**: `[θx, ωx, δx, θy, ωy, δy]` (X and Y axis states)
- **Inputs**: X and Y TVC commands
- **Outputs**: X and Y angles and angular velocities

### State-Space Representation
```
ẋ = Ax + Bu
y = Cx + Du
```

Where:
- **A**: System dynamics matrix
- **B**: Input matrix
- **C**: Output matrix
- **D**: Feedthrough matrix

## LQR Controller Design

### Theory
The LQR controller minimizes the cost function:
```
J = ∫₀^∞ (x'Qx + u'Ru) dt
```

### Default Weighting Matrices
- **Q**: Identity matrix (equal weighting on all states)
- **R**: Identity matrix (standard input weighting)

### Gain Calculation
The optimal gain K is computed by solving the Algebraic Riccati Equation (ARE):
```
A'P + PA - PBR⁻¹B'P + Q = 0
K = R⁻¹B'P
```

## Tuning Guide

### 1. Physical Parameters
Start with accurate physical measurements:
- Measure rocket mass and moment of inertia
- Determine TVC-to-COM distance precisely
- Characterize actuator dynamics (delay, rate limits)

### 2. LQR Tuning
Modify the Q and R matrices in the `design_lqr()` function:

**For faster response:**
```python
Q = np.diag([10, 1, 0.1])  # Higher angle weighting
R = np.eye(1)              # Standard input weighting
```

**For smoother control:**
```python
Q = np.eye(3)              # Standard state weighting  
R = np.array([[10]])       # Higher input penalty
```

**State weighting priorities:**
- `Q[0,0]`: Angle error penalty
- `Q[1,1]`: Angular velocity penalty  
- `Q[2,2]`: TVC deflection penalty

### 3. Frequency Domain Considerations
- Ensure control frequency > 10× system bandwidth
- Account for actuator delays in stability margins
- Consider sensor noise filtering requirements

### 4. Comparison with PID
Use the built-in PID comparison to validate LQR performance:
- LQR should show better transient response
- PID may have steady-state advantages with integral action
- Compare settling time, overshoot, and control effort

## Output Interpretation

### Plots
The system generates time-domain plots showing:
- **Angle Response**: θ(t) for both LQR and PID
- **Comparison**: Side-by-side performance evaluation

### Performance Metrics
Evaluate controllers based on:
- **Settling Time**: Time to reach steady-state
- **Overshoot**: Maximum deviation from final value
- **Rise Time**: Time to reach target
- **Control Effort**: Magnitude of control inputs

## Troubleshooting

### Common Issues

**1. Unstable Response**
- Check physical parameters (especially moment of inertia)
- Reduce Q matrix values
- Increase R matrix values
- Verify actuator limits

**2. Slow Response**
- Increase Q matrix diagonal values
- Reduce R matrix values
- Check control frequency settings

**3. Oscillatory Behavior**
- Reduce derivative gains in Q matrix
- Add damping to the model
- Check for unmodeled delays

**4. Configuration Errors**
- Validate JSON syntax
- Ensure all required parameters are present
- Check units consistency

### Debug Tips
- Start with conservative gains
- Test with step inputs first
- Monitor control saturation
- Validate model assumptions

## Advanced Usage

### Custom Weighting Matrices
Modify the `design_lqr()` function call:
```python
# Custom Q matrix - emphasize angle control
Q_custom = np.diag([100, 10, 1])
R_custom = np.array([[1]])
Kc = design_lqr(sys_c.A, sys_c.B, Q_custom, R_custom)
```

### Different Reference Inputs
The system currently supports step references. For custom trajectories, modify the `simulate_response()` function.

### Model Extensions
- Add integrator states for zero steady-state error
- Include disturbance models
- Implement gain scheduling for nonlinear operation

## References

- Linear Quadratic Regulator theory
- Rocket control system design principles  
- State-space control methods

## License

[Add your license information here] 