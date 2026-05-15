# A.E.O.L.U.S Flight Controller

> **Adaptable & Extensible Open-Loop Unmanned System**  
> Quadcopter Firmware — Bottom-Up Design Approach  
> *Robotics Society, Indian Institute of Technology Jodhpur*

---

## Overview

A.E.O.L.U.S is a fully customisable quadcopter flight controller firmware built using a **bottom-up design methodology** — starting from first-principles rigid-body physics and progressively layering sensor fusion, control allocation, and PID-based attitude/altitude control. The full system is modelled and validated in **MATLAB Simulink**.

---

## System Architecture

The system is composed of three interconnected subsystems:

```
┌─────────────┐     motor RPSs     ┌───────────┐     body-frame signals     ┌─────────────────┐
│  Controller │ ─────────────────► │   Plant   │ ──────────────────────────► │  Estimation &   │
│  (PID loops)│                    │ (Physics) │                             │    Sensing      │
└─────────────┘ ◄────────────────── └───────────┘                            └─────────────────┘
                  estimated states                                            (complementary filter)
```

| Block | Role | Key Outputs |
|---|---|---|
| **Plant** | Simulates quadcopter rigid-body physics | Body-frame accelerations, angular rates, magnetometer readings |
| **Estimation & Sensing** | Discretises plant outputs; sensor fusion via complementary filter | Estimated roll, pitch, yaw |
| **Controller** | Joint attitude + altitude PID; motor mix allocation | Motor RPS commands |

---

## Plant Model

All dynamics are formulated in continuous time and subsequently discretised.

### Thrust Generation
Each motor *i* produces thrust proportional to the square of its angular velocity:

```
Tᵢ = b · ωᵢ²      (i = 1, 2, 3, 4)
```
where `b` is the thrust coefficient and `ωᵢ` is the angular speed of motor *i*.

### Translational Dynamics
```
m·v̈ = [0, 0, mg]ᵀ − R_BW · [0, 0, T]ᵀ
```

### Rotational Dynamics
```
Roll torque:   τx = d(T₁ − T₂) = db(ω₁² − ω₂²)
Pitch torque:  τy = d(T₃ − T₄) = db(ω₃² − ω₄²)
Yaw torque:    τu = k(ω₁² + ω₃² − ω₂² − ω₄²)

J·ω̇ = −ω × Jω + Γ
```

### Plant Outputs
- Body-frame linear acceleration: `aB = [u̇, v̇, ẇ]ᵀ`
- Body-frame angular velocity: `ωB = [p, q, r]ᵀ`
- Body-frame magnetometer: `mB = R_BWᵀ · m_w`

---

## Control Allocation (Motor Mix)

The motor mix block converts desired torques `(τx, τy, τu)` and total thrust `T` into individual motor angular speeds using the control allocation matrix **A**:

|       | ω₁²  | ω₂²  | ω₃²  | ω₄²  |
|-------|------|-------|------|-------|
| **T** | b    | b     | b    | b     |
| **τx**| 0    | −bd   | 0    | bd    |
| **τy**| −db  | 0     | db   | 0     |
| **τu**| k    | −k    | k    | −k    |

The inverse **A⁻¹** exists provided `b, k, d > 0`.

---

## Estimation & Sensing

The estimation block discretises plant outputs via rate transition and fuses gyroscope, accelerometer, and magnetometer data through a **complementary filter**.

### Gyroscope Integration
```
roll_gyro(i)  = prev_roll  + gx · dt
pitch_gyro(i) = prev_pitch + gy · dt
yaw_gyro(i)   = prev_yaw   + gz · dt
```
*Precise over short intervals but accumulates drift over time.*

### Accelerometer Estimation
```
roll_acc  = arctan(ay / az)
pitch_acc = arctan(−ax / √(ay² + az²))
```
*Noise-sensitive and unreliable during dynamic manoeuvres.*

### Tilt-Compensated Magnetometer
```
m'x    = mx·cos(pitch_acc) + mz·sin(pitch_acc)
m'y    = mx·sin(roll_acc)·sin(pitch_acc) + my·cos(roll_acc) − mz·sin(roll_acc)·cos(pitch_acc)
yaw_mag = arctan(−m'y / m'x)
```

### Complementary Filter
Fuses the fast-but-drifting gyroscope with stable sensor estimates using high-pass coefficient **α = 0.9995**:

```
Pᵢ = α · P_gyro + (1 − α) · P_acc
```

Applies to roll & pitch (accelerometer-corrected) and yaw (magnetometer-corrected).

---

## Controller Design

The controller implements a **joint attitude and altitude** control framework. Pitch, roll, and yaw are treated as independent PID loops.

### Attitude PID
```
τ_θ(t) = Kp·eθ(t) + Ki·∫eθ(τ)dτ + Kd·(deθ/dt)
```

### Altitude PID with Tilt Compensation
```
Altitude error:  eu     = z_ref − z_est
PID thrust:      T_PID  = PID_z(eu)
Total thrust:    T_total = (mg + T_PID) · sec(θp) · sec(θr)
Motor thrust:    T_thrust = T_total  (if T_total ≥ 0, else 0)
```

The `sec(θp)·sec(θr)` term maintains vertical thrust regardless of attitude.

### Control Architecture Summary

| Loop | Input | Output | Method |
|---|---|---|---|
| Roll PID | Ref. roll, estimated roll | Roll torque τx | PID |
| Pitch PID | Ref. pitch, estimated pitch | Pitch torque τy | PID |
| Yaw PID | Ref. yaw, estimated yaw | Yaw torque τu | PID |
| Altitude PID | Ref. altitude, estimated alt. | Thrust T (tilt-compensated) | PID + sec(θ) |
| Motor Mix | τx, τy, τu, T | ω₁, ω₂, ω₃, ω₄ | A⁻¹ allocation |

---

## Simulink Implementation

The complete system is modelled in MATLAB Simulink using a hierarchical block structure. Each subsystem is encapsulated separately and connected by bus signals.

- **Plant subsystem** — integrates translational and rotational dynamics using a fixed-step `ode4` (Runge-Kutta) solver
- **Rate Transition blocks** — bridge continuous plant outputs and discrete sensor/estimation logic running at the IMU sample rate
- **Controller subsystem** — independent PID blocks for pitch, roll, and yaw, plus the altitude PID with tilt-compensation MATLAB Function
- **Motor Mix subsystem** — implements A⁻¹ allocation to convert control torques to per-motor speed commands

---

## Simulation Results

The model was validated against step and dynamic (gamepad) reference inputs:

| Test Scenario | Observation |
|---|---|
| Step altitude reference | Drone climbs to target; attitude-invariant thrust maintains level flight during ascent |
| Simultaneous yaw + altitude | Altitude and yaw channels show decoupled tracking; small cross-coupling transients decay quickly |
| Dynamic reference altitude (gamepad) | Controller tracks smoothly varying reference; tilt compensation effective across attitude envelope |
| Motor power distribution | All four motors operate within valid RPM bounds; allocation matrix correctly distributes torques |

The complementary filter successfully corrects gyroscope drift while rejecting high-frequency accelerometer noise, producing stable attitude estimates throughout simulation.

---

## Future Work

- Hardware-in-the-loop (HIL) testing on a real flight controller
- PID gain tuning for physical motor/ESC dynamics
- GPS-based position hold integration

---

## Repository Structure *(suggested)*

```
AEOLUS/
├── simulink/
│   ├── top_level.slx          # Top-level Simulink model
│   ├── plant/                 # Plant subsystem
│   ├── controller/            # PID controller subsystem
│   └── estimation/            # Sensor fusion subsystem
├── matlab/
│   └── params.m               # System parameters (b, k, d, m, J, etc.)
├── docs/
│   └── Technical_Report.pdf   # Full technical report
└── README.md
```

---

## License

This project was developed by the **Robotics Society, IIT Jodhpur**. Refer to individual contributors for licensing details.
