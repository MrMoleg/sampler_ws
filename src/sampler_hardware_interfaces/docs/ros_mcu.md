# Hardware Interface Implementation

## 1. Overview

The Hardware Interface (HwI) sends commands through the CAN bridge, which forwards them over the CAN bus to the motor drivers.

The HwI supports three types of motor commands:

- **`set_pos`** – target position (meters or radians).
- **`set_RPM`** – target rotational speed (RPM).
- **`set_duty`** – motor duty cycle (for manual motor control).

### Control modes

- **`set_pos`** – used for joints controlled by a position PID controller (mainly autonomous operation).
- **`set_RPM`** – used for joints controlled by a speed PID controller (autonomous operation; potentially manual control).
- **`set_duty`** – used for direct motor control, mainly during manual operation. Can be used instead of `set_RPM`.

The HwI selects the appropriate command based on the current `rover_status`. The motor driver changes its control mode according to the received command.

**Only one control mode can be active for a given motor at a time.**

## 2. Communication and Control Loop

The HwI operates in an open-loop manner: it forwards the requested position or speed increments without performing its own motion-control feedback correction.

It runs at **20 Hz**, sending commands over CAN every 50 ms and receiving feedback from the motor drivers.

### Command structure

A command for a single motor contains:

| Field | Description |
|---|---|
| `actuator_id` | Motor/actuator identifier |
| `command_id` | Command type (`set_pos`, `set_RPM`, `set_duty`) |
| `set_value` | Target value |

Example:

```text
actuator_id = <platform_id>
command_id  = <set_pos>
set_value   = 0.4  // meters
```

## 3. Joint Command Interfaces

Possible command interfaces for each sampler joint:

| Joint | Position | Velocity | Duty |
|---|:---:|:---:|:---:|
| `PLATFORM` | ✓ | ✓ | ✓ |
| `DRILL` | ✓ | ✓ | ✓ |
| `CONTAINER` | ✓ | ✓ | ✓ |
| `DRILL_ROTOR` | – | ✓ | ✓ |
| `VACUUM_ROTOR` | – | ✓ | ✓ |
| `BRUSH_ROTOR` | – | ✓ | ✓ |
| `CLAMP_JOINT` | ✓ | ✓ | ✓ |

**Note:** The clamp may be better represented as a discrete open/closed state rather than a continuous position, depending on its mechanical and electrical implementation.

## 4. Feedback Interfaces

Feedback is primarily used for monitoring joint states and providing information to higher-level controllers.

Possible feedback interfaces:

| Joint | Position | Velocity | Effort |
|---|:---:|:---:|:---:|
| `PLATFORM` | ✓ | ✓ | – |
| `DRILL` | ✓ | ✓ | – |
| `CONTAINER` | ✓ | ✓ | – |
| `DRILL_ROTOR` | – | ✓ | ✓ |
| `VACUUM_ROTOR` | – | ✓ | – |
| `BRUSH_ROTOR` | – | ✓ | – |
| `CLAMP_JOINT` | ✓ | – | – |

For the clamp, feedback could alternatively be represented as a discrete state.

## 5. General Sampler Feedback

The following information is expected from the sampler:

| Field | Type | Description |
|---|---|---|
| `actuator_id` | `uint8[]` | Actuator identifiers |
| `positions` | `float64[]` | Joint positions |
| `velocities` | `float64[]` | Joint velocities |
| `end_switches` | `bool[]` | End-switch states, if available |
| `rotor_effort` | `float64` | Main rotor effort/current |
| `distance_sensor` | `float64` | Distance sensor measurement |

**Important:** The distance sensor is particularly useful for position calibration during autonomous operation.

The feedback arrays should use a consistent actuator ordering or explicitly associate each measurement with its `actuator_id`. (ROS part)




