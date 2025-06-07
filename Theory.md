# LQR Rocket Control Theory

A comprehensive guide to understanding the Linear Quadratic Regulator (LQR) implementation for rocket attitude control, including state-space construction and block diagram analysis.

## Table of Contents
1. [System Overview](#system-overview)
2. [State-Space Model Construction](#state-space-model-construction)
3. [Building Block Analysis](#building-block-analysis)
4. [LQR Controller Design](#lqr-controller-design)
5. [Block Diagram Representation](#block-diagram-representation)
6. [Configuration Parameters as Blocks](#configuration-parameters-as-blocks)

## System Overview

The rocket attitude control system is designed around a **linearized single-axis model** that captures the essential dynamics of rocket stabilization. The system uses three states to represent the complete dynamics:

- **θ (theta)**: Rocket tilt angle [rad]
- **ω (omega)**: Angular velocity [rad/s]  
- **δ (delta)**: TVC deflection angle [rad]

## State-Space Model Construction

### Physical Derivation

The state-space matrices are derived from first principles of rocket dynamics:

#### **A Matrix - System Dynamics**
```
A = [[ 0,  1,  0 ]
     [ 0,  0,  k ]
     [ 0,  0, -1/τ]]
```

**Row 1: θ̇ = ω**
- The angle derivative is simply the angular velocity
- `A[0,1] = 1` represents this kinematic relationship

**Row 2: ω̇ = k·δ**
- Angular acceleration is proportional to TVC deflection
- `k = T₀·l/I` is the **control effectiveness**
  - `T₀`: Constant thrust [N]
  - `l`: Distance from TVC to center of mass [m]
  - `I`: Moment of inertia [kg·m²]
- `A[1,2] = k` represents the torque generation

**Row 3: δ̇ = -δ/τ + u/τ**
- TVC actuator modeled as first-order system
- `τ`: Servo time constant [s]
- `A[2,2] = -1/τ` represents actuator dynamics

#### **B Matrix - Input Coupling**
```
B = [[ 0   ]
     [ 0   ]
     [ 1/τ ]]
```

- Only the actuator state is directly affected by control input
- `B[2,0] = 1/τ` represents the actuator gain

#### **C Matrix - Sensor Outputs**
```
C = [[ 1,  0,  0 ]
     [ 0,  1,  0 ]]
```

- Measures angle and angular velocity
- `C[0,0] = 1`: Angle sensor
- `C[1,1] = 1`: Gyroscope (rate sensor)

#### **D Matrix - Feedthrough**
```
D = [[ 0 ]
     [ 0 ]]
```

- No direct feedthrough from input to output

### Configuration Parameter Mapping

Each config parameter directly affects the state-space matrices:

| Parameter | Symbol | Matrix Element | Physical Meaning |
|-----------|--------|----------------|------------------|
| `moment_of_inertia` | I | A[1,2] = T₀·l/I | Rotational inertia |
| `tvc_to_COM` | l | A[1,2] = T₀·l/I | Moment arm |
| `constant_thrust_N` | T₀ | A[1,2] = T₀·l/I | Thrust magnitude |
| `servo_delay_sec` | τ | A[2,2], B[2,0] | Actuator bandwidth |

## Building Block Analysis

### Block 1: Rocket Rigid Body Dynamics
```
Config: moment_of_inertia, tvc_to_COM, constant_thrust_N
Transfer Function: θ(s)/F(s) = (T₀·l)/(I·s²)
```

**Implementation:**
```python
I = config.get('moment_of_inertia', 0.00469)
l = config.get('tvc_to_COM', 0.147)
T0 = config.get('constant_thrust_N', 3.5)
k = T0 * l / I  # Control effectiveness
```

**Physical Meaning:**
- Double integrator representing rotational dynamics
- Gain `k` determines how effectively TVC deflection creates torque
- Higher thrust or longer moment arm → higher control authority
- Higher inertia → lower control authority

### Block 2: Servo Actuator Dynamics
```
Config: servo_delay_sec
Transfer Function: δ(s)/u(s) = 1/(τ·s + 1)
```

**Implementation:**
```python
tau = config.get('servo_delay_sec', 0.03333)
A[2,2] = -1/tau
B[2,0] = 1/tau
```

**Physical Meaning:**
- First-order lag representing servo response
- Time constant τ = 33ms typical for hobby servos
- Bandwidth ≈ 1/(2π·τ) ≈ 4.8 Hz
- Limits control system bandwidth

### Block 3: Sensor Block
```
Config: sensor_frequency_hz (implicit)
Transfer Function: y(s) = C·x(s)
```

**Implementation:**
```python
C = np.array([[1, 0, 0], [0, 1, 0]])  # Angle and rate sensors
```

**Physical Meaning:**
- Perfect sensors (no dynamics modeled)
- Could be extended to include sensor noise, bias, delays

### Block 4: Actuator Constraints
```
Config: tvc_angle_range, tvc_motor_offset_x_deg
Nonlinear Block: Saturation + Bias
```

**Implementation:**
```python
def apply_servo_constraints(u_cmd, config, dt):
    tvc_angle_deg = np.degrees(u_cmd)
    max_tvc_angle = config.get('tvc_angle_range', 8.0)
    tvc_angle_deg = np.clip(tvc_angle_deg, -max_tvc_angle, max_tvc_angle)
    motor_offset = config.get('tvc_motor_offset_x_deg', 0.0)
    tvc_angle_deg += motor_offset
    return np.radians(tvc_angle_deg)
```

**Physical Meaning:**
- Saturation: ±8° TVC angle limits
- Bias: 0.7° calibration error
- Nonlinear effects not captured in linear model

### Block 5: Digital Control Implementation
```
Config: servo_frequency_hz
Discrete-Time Conversion: ZOH at 50Hz
```

**Implementation:**
```python
fs = config.get('servo_frequency_hz', 50)
A_d, B_d, C_d, D_d, _ = cont2discrete((A, B, C, D), 1/fs, method='zoh')
```

**Physical Meaning:**
- Zero-order hold discretization
- 50Hz control rate (20ms sample time)
- Introduces additional phase lag

## LQR Controller Design

### Cost Function
The LQR controller minimizes:
```
J = ∫₀^∞ (x'Qx + u'Ru) dt
```

Where:
- **Q**: State weighting matrix (penalizes deviations)
- **R**: Input weighting matrix (penalizes control effort)

### Default Weighting
```python
Q = np.eye(3)  # Equal weighting: [θ, ω, δ]
R = np.eye(1)  # Standard input penalty
```

### Algebraic Riccati Equation
```
A'P + PA - PBR⁻¹B'P + Q = 0
K = R⁻¹B'P
```

**Implementation:**
```python
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.solve(R, B.T @ P)
```

### Gain Interpretation
For typical rocket parameters, LQR gains are approximately:
- `K[0]`: Angle feedback gain ≈ 1.0
- `K[1]`: Rate feedback gain ≈ 1.0  
- `K[2]`: TVC feedback gain ≈ 2.0

## Block Diagram Representation

### Complete System Block Diagram
```
Reference → [Controller] → [Constraints] → [Servo] → [Rocket] → Output
    ↑             ↓            ↓           ↓         ↓        ↓
    └─────────── [Sensors] ←──────────────────────────────────┘
```

### Detailed Block Breakdown

#### 1. **Reference Input Block**
- **Input**: Desired angle (typically 0°)
- **Output**: Reference signal
- **Config**: None (hardcoded to 0)

#### 2. **Controller Block** 
- **Input**: State vector [θ, ω, δ]
- **Output**: Control command u
- **LQR**: `u = -K·x`
- **PID**: `u = -(Kp·θ + Kd·ω + Ki·∫θdt)`
- **Config**: `STAB_KP`, `STAB_KD`, `STAB_KI`

#### 3. **Actuator Constraints Block**
- **Input**: Control command u
- **Output**: Constrained command u_actual
- **Function**: Saturation + bias
- **Config**: `tvc_angle_range`, `tvc_motor_offset_x_deg`

#### 4. **Servo Dynamics Block**
- **Input**: Constrained command
- **Output**: Actual TVC deflection δ
- **Transfer Function**: `1/(τs + 1)`
- **Config**: `servo_delay_sec`

#### 5. **Rocket Dynamics Block**
- **Input**: TVC deflection δ
- **Output**: Angle θ, rate ω
- **Transfer Function**: `k/s²` (double integrator)
- **Config**: `moment_of_inertia`, `tvc_to_COM`, `constant_thrust_N`

#### 6. **Sensor Block**
- **Input**: Physical states [θ, ω]
- **Output**: Measured states [θ_meas, ω_meas]
- **Transfer Function**: Identity (perfect sensors)
- **Config**: Could add noise, bias, filtering

## Configuration Parameters as Blocks

### Physical Parameters
```json
{
    "moment_of_inertia": 0.00469,     // Rocket dynamics block
    "tvc_to_COM": 0.147,              // Rocket dynamics block  
    "constant_thrust_N": 3.5,         // Rocket dynamics block
    "mass": 0.292                     // Not used in attitude control
}
```

### Actuator Parameters
```json
{
    "servo_delay_sec": 0.03333,       // Servo dynamics block
    "tvc_angle_range": 8.0,           // Constraint block
    "tvc_motor_offset_x_deg": 0.7,    // Constraint block
    "tvc_deg_per_second_max": 125.0   // Rate limiter (not implemented)
}
```

### Control Parameters
```json
{
    "servo_frequency_hz": 50,         // Digital implementation block
    "STAB_KP": 0.15,                  // PID controller block
    "STAB_KD": 0.055,                 // PID controller block
    "STAB_KI": 0.08,                  // PID controller block
    "INTEGRAL_WINDUP_LIMIT_ANGLE": 3.0 // PID anti-windup block
}
```

### Simulation Parameters
```json
{
    "sim_time_seconds": 6,            // Simulation time
    "initial_x_rot_angle": 20,        // Initial condition
    "delta_t": 0.002                  // Not used (servo_frequency_hz sets dt)
}
```

## Advanced Block Implementation

### Missing Blocks (Could Be Added)

#### 1. **Rate Limiter Block**
```python
def apply_rate_limit(u_cmd, u_prev, dt, max_rate):
    max_change = max_rate * dt
    du = u_cmd - u_prev
    du_limited = np.clip(du, -max_change, max_change)
    return u_prev + du_limited
```

#### 2. **Sensor Noise Block**
```python
def add_sensor_noise(y_true, config):
    noise_std = config.get('angle_sensor_noise_std', 0.0)
    bias = config.get('angle_sensor_bias', 0.0)
    noise = np.random.normal(0, noise_std, y_true.shape)
    return y_true + noise + bias
```

#### 3. **Anti-Windup Block**
```python
def anti_windup(integral, u_cmd, u_actual, ki, dt):
    # Back-calculation anti-windup
    if ki > 0:
        integral -= (u_cmd - u_actual) / ki * dt
    return integral
```

## Simulink Equivalent Blocks

If implementing in Simulink, the system would use:

1. **Integrator blocks** (2x) for rocket dynamics
2. **Transfer Function block** for servo (1/(τs+1))
3. **Gain blocks** for control effectiveness k
4. **Saturation block** for TVC limits
5. **Bias block** for motor offset
6. **Sum blocks** for error calculation
7. **Matrix Gain block** for LQR controller
8. **PID Controller block** for PID comparison
9. **Zero-Order Hold** for digital implementation
10. **Scope blocks** for visualization

This modular approach allows easy modification and understanding of each system component's contribution to overall performance.

## References

- Linear Quadratic Regulator theory
- Rocket control system design principles  
- State-space control methods

## License

[Add your license information here] 