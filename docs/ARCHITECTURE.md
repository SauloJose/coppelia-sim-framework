
# Brainbyte Architecture & System Design

This document outlines the architectural decisions, communication protocols, and system design principles underlying the Brainbyte framework. It is intended for developers who want to understand *how* the framework operates under the hood.

---

## 1. The Communication Paradigm

The traditional approach to controlling CoppeliaSim via RemoteAPI involves sending a network request for every single variable read or motor command. In complex robots (e.g., reading a 360° LiDAR, a camera, and commanding multiple joints), this creates a massive network bottleneck, dropping simulation performance drastically.

Brainbyte solves this by completely replacing the classic RemoteAPI client with a custom **Synchronous Batch Dataflow Server**.

### 1.1 ZeroMQ (ZMQ) Sockets


The framework uses ZeroMQ as its transport layer, implementing a strict **REQ/REP (Request-Reply)** pattern:
* **Python Client (REQ):** The `SimulationBridge` in Python drives the clock. It sends a batch of commands and blocks execution until it receives the updated world state.
* **CoppeliaSim Server (REP):** A threaded Lua script inside the simulator listens on port `23001`. It receives the command batch, applies all actuations, steps the physics engine exactly once, and replies with all requested sensor data.

This guarantees that Python logic and CoppeliaSim physics are perfectly synchronized, regardless of the complexity of the Python algorithms.

### 1.2 CBOR2 Binary Serialization

While JSON is standard for web APIs, it is highly inefficient for robotics. Parsing massive text strings for high-density data (like Point Clouds or Vision matrices) causes severe CPU lag.

Brainbyte uses **CBOR (Concise Binary Object Representation)**. It supports the same dictionary structures as JSON but encodes everything into raw binary.
* **The "Zero-Copy" Advantage:** When CoppeliaSim sends a LiDAR point cloud, it packs it as raw bytes. CBOR transmits this natively. Upon receipt, the `SimulationBridge` maps these bytes directly into a `numpy.float32` array. This bypasses expensive string-to-float conversions entirely, keeping execution times in the microsecond range.

---

## 2. The Batch Dataflow Process

Instead of immediate execution, Brainbyte uses a deferred execution model.

1. **Queueing Phase:** During the `BaseApp.loop(t)` execution, calls to `self.bridge.queue_velocity()` do *not* touch the network. They simply append data to an internal Python dictionary (the "Batch").
2. **Step Phase:** Once the loop iteration ends, the `SimulationBridge` packs the entire Batch into a single CBOR payload and fires it over ZMQ.
3. **Cache Update:** The reply from CoppeliaSim contains a flat dictionary of all sensor states. This dictionary overwrites the local `latest_state` cache.
4. **Read Phase:** In the next loop iteration, when a robot class requests sensor data via `get_sensor_data()`, it simply reads from the local Python cache (an O(1) lookup with zero network overhead).

### 2.1 Payload Anatomy

The communication relies on specific dictionary structures encoded in CBOR. Here is how they look conceptually:

**1. INIT Payload (Handshake - Python -> CoppeliaSim):**
Sent exactly once before the simulation starts. It provides the Lua script with a list of all object paths that Python intends to monitor or control. The Lua engine finds their integer handles and caches them internally, avoiding slow string lookups during the main loop.
```json
{
    "type": "INIT",
    "paths": [
        "/TurtleBot/leftMotor",
        "/TurtleBot/rightMotor",
        "/TurtleBot/GPS",
        "/TurtleBot/LiDAR_bin"
    ]
}
```

**2. Outgoing STEP Payload (Python -> CoppeliaSim):**
Sent every frame. Contains the batched commands accumulated during the Python loop.
```json
{
    "type": "STEP",
    "velocities": {
        "/TurtleBot/leftMotor": 1.5, 
        "/TurtleBot/rightMotor": 1.5
    },
    "positions": {
        "/RoboticArm/Joint1": 0.785
    }
}
```

**3. Incoming STEP Payload (CoppeliaSim -> Python):**
Received every frame. Notice that paths are used as literal dictionary keys. This flat structure allows Python to update its cache instantly.
```json
{
    "/TurtleBot/GPS": [1.2, 0.5, 0.0],
    "/TurtleBot/LiDAR_bin": <binary_bytes>
}
```

---

## 3. BaseApp Execution Lifecycle

The `BaseApp` class is a State Machine that guarantees safe startup and teardown of the simulation environment.

```text
[ RUN INVOKED ]
      │
      ▼
1. INITIALIZATION
      ├─ Load configuration and instantiate loggers.
      ├─ Connect to ZMQ Socket via SimulationBridge.
      └─ Load the specified `.ttt` scene into CoppeliaSim.
      │
      ▼
2. SETUP PHASE
      ├─ Execute user-defined `setup()` (get handles, declare variables).
      └─ Bridge sends 'INIT' Payload. CoppeliaSim caches the handles.
      │
      ▼
3. SIMULATION START
      ├─ Trigger CoppeliaSim to start the physics engine.
      ├─ Execute user-defined `post_start()` (capture initial poses/diagnostics).
      └─ Perform first data fetch to populate the sensor cache.
      │
      ▼
4. MAIN LOOP (Synchronous Stepping)
      │  ┌───────────────────────────────────────────────┐
      │  │ a. Check for user interrupts (Ctrl+C / 's').  │
      ├──┼─┤ b. Execute user-defined `loop(t)`.          │
      │  │ c. Bridge sends queued commands (STEP).       │
      │  │ d. CoppeliaSim steps physics and replies.     │
      │  └───────────────────────────────────────────────┘
      │    (Loops while t < sim_time and no interrupt)
      ▼
5. TEARDOWN
      ├─ Stop physics simulation in CoppeliaSim.
      ├─ Execute user-defined `stop()` (plot data, save logs).
      └─ Gracefully close ZMQ sockets and network ports.
```

---

## 4. Module Separation Philosophy

The folder structure enforces a strict separation of concerns (SoC):

* **`core/`**: The unchangeable engine. It handles time, network, and logging. Code here is completely agnostic to *what* robot is being simulated.
* **`robots/`**: The physical definitions. Classes here know the paths, kinematics, and constraints of specific hardware (e.g., `TurtleBot` knows its wheel radius and track width), but they do not manage the ZMQ connection directly.
* **`utils/`**: Stateless helper functions (math conversions, matplotlib wrappers) that have zero side effects on the simulation state.
* **`projects/`**: The domain logic. This is where the user resides, writing scripts that tie `core`, `robots`, and `utils` together to solve specific challenges (like locomotion or obstacle avoidance). 
