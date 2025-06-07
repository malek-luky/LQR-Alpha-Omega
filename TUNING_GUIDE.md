# Rocket Control Tuning Guide

A comprehensive guide for tuning LQR and PID controllers for rocket attitude control, based on modern control theory principles.

## Table of Contents
1. [Control Theory Fundamentals](#control-theory-fundamentals)
2. [LQR Controller Tuning](#lqr-controller-tuning)
3. [PID Controller Tuning](#pid-controller-tuning)
4. [Practical Tuning Steps](#practical-tuning-steps)
5. [Performance Analysis](#performance-analysis)
6. [Troubleshooting](#troubleshooting)

## Control Theory Fundamentals

### System Model
The rocket attitude dynamics are modeled as a 3rd-order system:
```
States: x = [θ, ω, δ]ᵀ
- θ: Angle (rad)
- ω: Angular velocity (rad/s)  
- δ: TVC deflection (rad)
```

### State-Space Representation
```
ẋ = Ax + Bu
y = Cx

A = [0  1   0 ]    B = [  0  ]    C = [1 0 0]
    [0  0   k ]        [  0  ]        [0 1 0]
    [0  0 -1/τ]        [1/τ ]

where:
k = T₀ × l / I    (control effectiveness)
τ = servo time constant
```

## LQR Controller Tuning

### Theory
LQR minimizes the quadratic cost function:
```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

The optimal gain is: **K = R⁻¹BᵀP**

Where P solves the Algebraic Riccati Equation:
```
AᵀP + PA - PBR⁻¹BᵀP + Q = 0
```

### Q Matrix Design (State Weighting)

The Q matrix penalizes state deviations. For rocket control:

```python
Q = diag([q₁, q₂, q₃])
```

**Physical Interpretation:**
- `q₁`: Angle error penalty (most important for stability)
- `q₂`: Angular velocity penalty (damping)
- `q₃`: TVC deflection penalty (actuator usage)

### R Matrix Design (Control Weighting)

```python
R = [r₁]  # Single input system
```

**Trade-offs:**
- **Small R**: Aggressive control, fast response, high actuator usage
- **Large R**: Conservative control, slower response, low actuator usage

### Systematic LQR Tuning Procedure

#### Step 1: Baseline Design
Start with identity matrices:
```python
Q = np.eye(3)
R = np.array([[1]])
```

#### Step 2: Adjust Angle Weighting
Increase `q₁` for better angle tracking:
```python
Q = np.diag([10, 1, 1])    # Emphasize angle control
Q = np.diag([100, 1, 1])   # Very tight angle control
```

#### Step 3: Tune Damping
Adjust `q₂` for proper damping:
```python
Q = np.diag([10, 5, 1])    # More damping
Q = np.diag([10, 0.1, 1])  # Less damping (faster but oscillatory)
```

#### Step 4: Control Effort Limitation
Increase R to reduce actuator usage:
```python
R = np.array([[10]])       # Conservative control
R = np.array([[0.1]])      # Aggressive control
```

#### Step 5: Actuator Penalty
Adjust `q₃` to penalize TVC deflection:
```python
Q = np.diag([10, 1, 0.1])  # Allow large deflections
Q = np.diag([10, 1, 10])   # Penalize deflections
```

### Advanced LQR Tuning

#### Bryson's Rule
For initial Q matrix design:
```python
Q[i,i] = 1 / (maximum acceptable x[i])²
R[j,j] = 1 / (maximum acceptable u[j])²
```

Example for rocket:
```python
max_angle = 0.1      # 0.1 rad = ~6 degrees
max_rate = 1.0       # 1 rad/s
max_deflection = 0.1 # 0.1 rad TVC deflection

Q = np.diag([1/max_angle², 1/max_rate², 1/max_deflection²])
```

#### Frequency Domain Considerations
- **Bandwidth**: LQR bandwidth ≈ √(q₁/r₁)
- **Phase Margin**: Typically 45-60° for good robustness
- **Gain Margin**: Should be > 6 dB

## PID Controller Tuning

### PID as State Feedback
For comparison with LQR, PID is implemented as:
```
u = -Kp×θ - Kd×ω
```

This corresponds to state feedback: `u = -K×x` where `K = [Kp, Kd, 0]`

### Classical PID Tuning Methods

#### 1. Ziegler-Nichols Method
1. Set Ki = Kd = 0
2. Increase Kp until system oscillates (critical gain Kc)
3. Measure oscillation period Tc
4. Apply Z-N formulas:
   - Kp = 0.6 × Kc
   - Ki = 2×Kp / Tc  
   - Kd = Kp×Tc / 8

#### 2. Cohen-Coon Method
Better for systems with significant delay:
1. Obtain step response
2. Measure delay time and time constant
3. Apply C-C formulas

#### 3. Manual Tuning Procedure
1. **Start with P-only**: Increase Kp until slight overshoot
2. **Add D-term**: Increase Kd to reduce overshoot and improve stability
3. **Add I-term**: Small Ki to eliminate steady-state error (if needed)

### PID Tuning Guidelines for Rockets

#### Proportional Gain (Kp)
- **Too low**: Slow response, large steady-state error
- **Too high**: Overshoot, oscillations, instability
- **Typical range**: 0.1 - 2.0 for normalized system

#### Derivative Gain (Kd)  
- **Purpose**: Provides damping, improves stability
- **Too low**: Oscillatory response
- **Too high**: Noise amplification, actuator wear
- **Typical range**: 0.01 - 0.5

#### Integral Gain (Ki)
- **Purpose**: Eliminates steady-state error
- **Caution**: Can cause windup and instability
- **Recommendation**: Start with Ki = 0, add only if needed
- **Typical range**: 0 - 0.1

## Practical Tuning Steps

### 1. System Identification
Before tuning, verify your model:
```bash
# Test with small initial disturbance
python main.py config.json --angle 0.05 --T 3
```

### 2. LQR Tuning Workflow

#### Quick Start
```python
# In main.py, modify design_lqr call:
Q = np.diag([10, 1, 0.1])  # Start here
R = np.array([[1]])
K_lqr = design_lqr(sys_c.A, sys_c.B, Q, R)
```

#### Iterative Process
1. **Test baseline**: `Q = I, R = I`
2. **Increase angle weighting**: `Q[0,0] = 10, 50, 100`
3. **Adjust damping**: `Q[1,1] = 0.1, 1, 10`
4. **Tune control effort**: `R = 0.1, 1, 10`
5. **Fine-tune**: Small adjustments based on response

### 3. Performance Metrics

#### Time Domain
- **Settling time**: Time to reach ±2% of final value
- **Overshoot**: Maximum deviation from final value
- **Rise time**: Time to reach 90% of final value

#### Frequency Domain
- **Bandwidth**: Frequency where |H(jω)| = -3dB
- **Phase margin**: Phase at gain crossover frequency
- **Gain margin**: Gain at phase crossover frequency

### 4. Comparison Testing
```bash
# Test different initial angles
python main.py config.json --angle 0.1 --T 5
python main.py config.json --angle 0.2 --T 5

# Test different time scales
python main.py config.json --T 3   # Quick response
python main.py config.json --T 10  # Long-term behavior
```

## Performance Analysis

### Evaluating Controller Performance

#### 1. Stability
- All poles in left half-plane (continuous)
- All poles inside unit circle (discrete)
- Adequate stability margins

#### 2. Performance
- Fast settling time (< 2 seconds typical)
- Low overshoot (< 10%)
- Good disturbance rejection

#### 3. Robustness
- Insensitive to parameter variations
- Good performance across flight envelope
- Reasonable control effort

### LQR vs PID Comparison

| Aspect | LQR | PID |
|--------|-----|-----|
| **Design** | Systematic, optimal | Trial-and-error |
| **Tuning** | Q, R matrices | Kp, Ki, Kd gains |
| **Performance** | Guaranteed stability | May require careful tuning |
| **Robustness** | Good margins | Depends on tuning |
| **Implementation** | State feedback | Error feedback |

## Troubleshooting

### Common Issues

#### LQR Problems
- **Slow response**: Increase Q diagonal values
- **Oscillatory**: Reduce Q values or increase R
- **High control effort**: Increase R matrix
- **Poor tracking**: Increase Q[0,0] (angle weighting)

#### PID Problems  
- **Oscillations**: Reduce Kp, increase Kd
- **Slow response**: Increase Kp
- **Steady-state error**: Add small Ki (carefully)
- **Noise sensitivity**: Reduce Kd, add filtering

#### System Issues
- **Instability**: Check model parameters (especially moment of inertia)
- **Poor performance**: Verify actuator bandwidth and delays
- **Unrealistic behavior**: Check thrust and geometry values

### Debugging Commands
```bash
# Test system stability
python main.py config.json --angle 0.01 --T 2

# Check for oscillations  
python main.py config.json --angle 0.2 --T 8

# Test with different parameters
python main.py config.json --angle 0.15 --T 6
```

### Parameter Sensitivity
Test robustness by varying key parameters:
- ±20% moment of inertia
- ±50% servo delay
- ±30% thrust level

## Advanced Topics

### Gain Scheduling
For varying flight conditions:
```python
# Altitude-dependent gains
if altitude < 1000:
    Q = Q_low_altitude
else:
    Q = Q_high_altitude
```

### Integral Action for LQR
Add integrator states for zero steady-state error:
```python
# Augmented system: [θ, ω, δ, ∫θ]
A_aug = np.block([[A, np.zeros((3,1))], 
                  [C[0,:], 0]])
B_aug = np.vstack([B, 0])
```

### Kalman Filter Integration
Combine LQR with state estimation:
```python
# LQG = LQR + Kalman Filter
# Optimal for noisy measurements
```

## References

- **Brian Douglas Control Systems Lectures**: Excellent YouTube series on control theory
- **Modern Control Engineering** by Ogata: Comprehensive textbook
- **Optimal Control Theory** by Lewis & Syrmos: Advanced LQR theory
- **Rocketry Forum**: Practical discussions on rocket control systems

## Quick Reference

### Default Starting Values
```python
# LQR
Q = np.diag([10, 1, 0.1])
R = np.array([[1]])

# PID  
Kp = 0.15
Kd = 0.055
Ki = 0.0  # Start with zero
```

### Typical Performance Targets
- Settling time: < 2 seconds
- Overshoot: < 10%
- Control effort: < 50% of maximum
- Stability margins: PM > 45°, GM > 6dB 