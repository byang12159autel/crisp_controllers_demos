# CRISP Controllers ↔ crisp_mujoco_sim Data Exchange Architecture

This document shows the information/data exchange between CRISP controllers and the crisp_mujoco_sim package, comparing the current C++ in-process architecture with the proposed Python service-based architecture.

---

## Table of Contents
1. [Current Architecture: C++ In-Process](#current-architecture-c-in-process)
2. [Proposed Architecture: Python Service-Based](#proposed-architecture-python-service-based)
3. [Data Exchange Summary](#data-exchange-summary)
4. [Migration Path](#migration-path)

---

## Current Architecture: C++ In-Process

### High-Level System Overview

```mermaid
flowchart TB
    subgraph ROS2_CONTROL["ROS2 Control Framework"]
        CM[Controller Manager]
        
        subgraph CONTROLLERS["CRISP Controllers"]
            JTC[Joint Trajectory Controller]
            FC[Force Controller]
            GC[Gravity Compensation]
        end
        
        subgraph BROADCASTERS["State Broadcasters"]
            JSB[Joint State Broadcaster]
            RB[Robot Broadcaster]
        end
    end
    
    subgraph HW_INTERFACE["Hardware Interface Layer (crisp_mujoco_sim)"]
        SI[System Interface<br/>Simulator class]
        
        subgraph INTERFACES["Exported Interfaces"]
            CMD[Command Interfaces<br/>effort[0..n]]
            STATE[State Interfaces<br/>position[0..n]<br/>velocity[0..n]<br/>effort[0..n]]
        end
    end
    
    subgraph SIM_LAYER["Simulation Layer (same process)"]
        MJS[MuJoCoSimulator<br/>Singleton]
        
        subgraph BUFFERS["Shared Memory Buffers"]
            ECMD[eff_cmd<br/>mutex protected]
            PSTATE[pos_state<br/>vel_state<br/>eff_state<br/>mutex protected]
        end
        
        MJ[MuJoCo Physics Engine<br/>C++ API]
    end
    
    %% Controller to Interface connections
    CONTROLLERS -->|write commands| CMD
    STATE -->|read states| CONTROLLERS
    STATE -->|read states| BROADCASTERS
    
    %% Interface to buffers
    SI -->|write cycle| ECMD
    PSTATE -->|read cycle| SI
    
    %% Buffers to simulator
    ECMD -->|control callback| MJ
    MJ -->|sync states| PSTATE
    
    MJS -.manages.-> BUFFERS
    MJS -.runs.-> MJ
    
    CM -.orchestrates.-> CONTROLLERS
    CM -.orchestrates.-> BROADCASTERS
    CM -.read/write cycle.-> SI
    
    style ROS2_CONTROL fill:#e1f5ff
    style HW_INTERFACE fill:#fff4e1
    style SIM_LAYER fill:#f0f0f0
    style BUFFERS fill:#ffe1e1
```

### Detailed Data Flow (C++ In-Process)

```mermaid
sequenceDiagram
    participant Controller as CRISP Controller<br/>(e.g., Joint Trajectory)
    participant CM as Controller Manager
    participant SI as System Interface<br/>(Simulator)
    participant Buf as Shared Buffers<br/>(mutex protected)
    participant Sim as MuJoCo Simulator<br/>(separate thread)
    participant MJ as MuJoCo Engine
    
    Note over Controller,MJ: Control Loop Cycle (e.g., 1kHz)
    
    rect rgb(200, 230, 255)
        Note over CM,SI: READ Phase
        CM->>SI: read(time, period)
        SI->>Buf: try_lock(state_mutex)
        Buf-->>SI: pos_state, vel_state, eff_state
        SI->>SI: copy to m_positions, m_velocities, m_efforts
        SI-->>CM: return OK
        CM->>Controller: state interfaces updated
        Controller->>Controller: compute control (PID, trajectory tracking)
    end
    
    rect rgb(255, 230, 200)
        Note over CM,SI: WRITE Phase
        Controller->>CM: command interfaces (m_effort_commands[])
        CM->>SI: write(time, period)
        SI->>Buf: try_lock(command_mutex)
        SI->>Buf: eff_cmd = m_effort_commands
        SI-->>CM: return OK
    end
    
    rect rgb(230, 255, 230)
        Note over Sim,MJ: Simulation Thread (parallel)
        Sim->>MJ: mj_step1(m, d)
        MJ->>Sim: controlCB callback
        Sim->>Buf: lock(command_mutex)
        Buf-->>Sim: eff_cmd[]
        Sim->>MJ: d->ctrl[i] = eff_cmd[i]
        Sim->>MJ: continue mj_step1
        MJ->>MJ: Forward dynamics<br/>Integration
        Sim->>Buf: lock(state_mutex)
        Sim->>Buf: syncStates()<br/>pos_state = d->qpos<br/>vel_state = d->qvel<br/>eff_state = d->qfrc_actuator
        Sim->>Sim: Real-time sync (sleep)
        Sim->>MJ: mj_step2(m, d)
    end
```

### Thread Architecture (C++ In-Process)

```mermaid
graph TB
    subgraph PROCESS["Single Process: controller_manager"]
        subgraph THREAD1["Thread 1: Controller Manager"]
            CM[Controller Manager Loop]
            CTRL[Controllers Update]
            SI_RW[System Interface<br/>read/write calls]
        end
        
        subgraph THREAD2["Thread 2: MuJoCo Simulator"]
            SIM_LOOP[Simulation Loop]
            MJ_STEP[mj_step1/2]
            CTRL_CB[Control Callback]
            SYNC[syncStates]
        end
        
        subgraph SHARED["Shared Memory"]
            CMD_BUF[eff_cmd<br/>command_mutex]
            STATE_BUF[pos_state, vel_state, eff_state<br/>state_mutex]
        end
        
        CM --> CTRL --> SI_RW
        SI_RW <-->|direct function calls| CMD_BUF
        SI_RW <-->|direct function calls| STATE_BUF
        
        SIM_LOOP --> MJ_STEP --> CTRL_CB --> SYNC
        CMD_BUF <-->|mutex lock| CTRL_CB
        STATE_BUF <-->|mutex lock| SYNC
    end
    
    style PROCESS fill:#f9f9f9
    style SHARED fill:#ffe1e1
```

---

## Proposed Architecture: Python Service-Based

### High-Level System Overview

```mermaid
flowchart TB
    subgraph ROS2_CONTROL["ROS2 Control Framework"]
        CM[Controller Manager]
        
        subgraph CONTROLLERS["CRISP Controllers"]
            JTC[Joint Trajectory Controller]
            FC[Force Controller]
            GC[Gravity Compensation]
        end
        
        subgraph BROADCASTERS["State Broadcasters"]
            JSB[Joint State Broadcaster]
            RB[Robot Broadcaster]
        end
    end
    
    subgraph HW_INTERFACE["Hardware Interface Layer (crisp_mujoco_sim)"]
        SI[System Interface<br/>Simulator class<br/>C++]
        
        subgraph INTERFACES["Exported Interfaces"]
            CMD[Command Interfaces<br/>effort[0..n]]
            STATE[State Interfaces<br/>position[0..n]<br/>velocity[0..n]<br/>effort[0..n]]
        end
        
        subgraph CLIENTS["ROS2 Service Clients"]
            INIT_CLI[Init Service Client]
            STEP_CLI[Step Service Client]
        end
    end
    
    subgraph ROS2_COMM["ROS2 Communication Layer"]
        INIT_SRV[Init Service]
        STEP_SRV[Step Service]
    end
    
    subgraph PY_SIM["Python Simulation Node (separate process)"]
        PY_NODE[Python ROS2 Node]
        
        subgraph PY_HANDLERS["Service Handlers"]
            INIT_HAND[init_callback]
            STEP_HAND[step_callback]
        end
        
        MJ_PY[MuJoCo Python API<br/>mujoco.MjModel<br/>mujoco.MjData]
    end
    
    %% Controller to Interface connections
    CONTROLLERS -->|write commands| CMD
    STATE -->|read states| CONTROLLERS
    STATE -->|read states| BROADCASTERS
    
    %% System Interface to ROS2 services
    SI -->|on_init: async call| INIT_CLI
    SI -->|write: async call| STEP_CLI
    INIT_CLI -->|Init.srv| INIT_SRV
    STEP_CLI -->|Step.srv| STEP_SRV
    
    %% ROS2 to Python
    INIT_SRV -->|model_path| INIT_HAND
    STEP_SRV -->|effort_commands[]| STEP_HAND
    INIT_HAND -->|pos, vel, eff| INIT_SRV
    STEP_HAND -->|pos, vel, eff| STEP_SRV
    
    %% Python handlers to MuJoCo
    INIT_HAND -->|mj.load_model| MJ_PY
    STEP_HAND -->|d.ctrl = efforts<br/>mj.step(m, d)| MJ_PY
    MJ_PY -->|d.qpos, d.qvel| STEP_HAND
    
    PY_NODE -.manages.-> PY_HANDLERS
    
    CM -.orchestrates.-> CONTROLLERS
    CM -.orchestrates.-> BROADCASTERS
    CM -.read/write cycle.-> SI
    
    style ROS2_CONTROL fill:#e1f5ff
    style HW_INTERFACE fill:#fff4e1
    style PY_SIM fill:#e1ffe1
    style ROS2_COMM fill:#ffe1f5
```

### Detailed Data Flow (Python Service-Based)

```mermaid
sequenceDiagram
    participant Controller as CRISP Controller<br/>(e.g., Joint Trajectory)
    participant CM as Controller Manager
    participant SI as System Interface<br/>(Simulator C++)
    participant Step_Srv as Step Service<br/>(ROS2)
    participant Py_Node as Python Sim Node
    participant MJ as MuJoCo Python API
    
    Note over Controller,MJ: Initialization Phase (once)
    
    rect rgb(240, 240, 240)
        CM->>SI: on_init(hardware_info)
        SI->>Py_Node: Init.srv(model_path)
        Py_Node->>MJ: mj.MjModel.from_xml_path(model_path)
        Py_Node->>MJ: mj.MjData(model)
        Py_Node->>MJ: d.qpos = m.keyframe.qpos
        MJ-->>Py_Node: model and data created
        Py_Node-->>SI: Init.response(pos[], vel[], eff[], success)
        SI->>SI: initialize m_positions, m_velocities, m_efforts
        SI-->>CM: CallbackReturn::SUCCESS
    end
    
    Note over Controller,MJ: Control Loop Cycle (e.g., 1kHz)
    
    rect rgb(200, 230, 255)
        Note over CM,SI: READ Phase
        CM->>SI: read(time, period)
        Note over SI: Use previous Step response<br/>stored in m_positions, etc.
        SI-->>CM: return OK
        CM->>Controller: state interfaces updated
        Controller->>Controller: compute control (PID, trajectory tracking)
    end
    
    rect rgb(255, 230, 200)
        Note over CM,SI: WRITE Phase
        Controller->>CM: command interfaces (m_effort_commands[])
        CM->>SI: write(time, period)
        SI->>Step_Srv: Step.srv(effort_commands[])
        Step_Srv->>Py_Node: step_callback(effort_commands[])
        Py_Node->>MJ: d.ctrl[:] = effort_commands
        Py_Node->>MJ: mj.step(m, d)
        MJ->>MJ: Forward dynamics<br/>Integration
        MJ-->>Py_Node: simulation stepped
        Py_Node->>Py_Node: extract d.qpos, d.qvel, d.qfrc_actuator
        Py_Node-->>Step_Srv: Step.response(pos[], vel[], eff[])
        Step_Srv-->>SI: Step.response
        SI->>SI: store in m_positions, m_velocities, m_efforts
        SI-->>CM: return OK
    end
```

### Process Architecture (Python Service-Based)

```mermaid
graph TB
    subgraph PROCESS1["Process 1: controller_manager"]
        CM[Controller Manager]
        CTRL[CRISP Controllers]
        SI[System Interface C++]
        
        subgraph CLIENTS["Service Clients"]
            INIT_CLI[Init Client]
            STEP_CLI[Step Client]
        end
        
        CM --> CTRL --> SI
        SI --> INIT_CLI
        SI --> STEP_CLI
    end
    
    subgraph ROS2_MW["ROS2 Middleware (DDS/Cyclone)"]
        INIT_TOPIC[Init Service Topic]
        STEP_TOPIC[Step Service Topic]
    end
    
    subgraph PROCESS2["Process 2: python_mujoco_sim"]
        PY_NODE[Python ROS2 Node]
        
        subgraph SERVERS["Service Servers"]
            INIT_SRV[Init Server]
            STEP_SRV[Step Server]
        end
        
        subgraph PY_SIM["Python Simulation"]
            MJ_MODEL[mj.MjModel]
            MJ_DATA[mj.MjData]
            SIM_LOOP[Simulation Logic]
        end
        
        PY_NODE --> INIT_SRV
        PY_NODE --> STEP_SRV
        INIT_SRV --> MJ_MODEL
        STEP_SRV --> SIM_LOOP
        SIM_LOOP --> MJ_DATA
    end
    
    INIT_CLI <-->|async request/response| INIT_TOPIC
    STEP_CLI <-->|async request/response| STEP_TOPIC
    INIT_TOPIC <-->|network/IPC| INIT_SRV
    STEP_TOPIC <-->|network/IPC| STEP_SRV
    
    style PROCESS1 fill:#e1f5ff
    style PROCESS2 fill:#e1ffe1
    style ROS2_MW fill:#ffe1f5
```

---

## Data Exchange Summary

### Data Types Exchanged

| Direction | Data Type | Size | Description | Current Method | Proposed Method |
|-----------|-----------|------|-------------|----------------|-----------------|
| Controller → Simulator | `effort_commands[]` | n × float64 | Joint torque commands | Mutex-protected buffer | ROS2 Step.srv request |
| Simulator → Controller | `position[]` | n × float64 | Joint positions (qpos) | Mutex-protected buffer | ROS2 Step.srv response |
| Simulator → Controller | `velocity[]` | n × float64 | Joint velocities (qvel) | Mutex-protected buffer | ROS2 Step.srv response |
| Simulator → Controller | `effort[]` | n × float64 | Joint efforts (qfrc_actuator) | Mutex-protected buffer | ROS2 Step.srv response |
| Controller → Simulator | `model_path` | string | Path to MuJoCo XML (init only) | Hardware parameter | ROS2 Init.srv request |

Where `n` = number of actuated joints (e.g., 7 for Franka FR3)

### Communication Patterns

#### Current (C++ In-Process)
- **Type**: Direct memory access via mutex-protected buffers
- **Latency**: < 1 microsecond (in-process)
- **Synchronization**: Mutexes + `try_lock()` for non-blocking
- **Threading**: Two threads in same process
- **Coupling**: Tight - crash in simulator crashes entire system
- **Language**: C++ only

#### Proposed (Python Service-Based)
- **Type**: ROS2 service calls (async request/response)
- **Latency**: ~100-500 microseconds (IPC, depends on DDS)
- **Synchronization**: ROS2 executor handles service dispatch
- **Threading**: Separate processes
- **Coupling**: Loose - processes isolated, can restart independently
- **Language**: C++ System Interface, Python simulator

### Service Definitions

#### Init.srv
```
# Request
string model_path

---
# Response
float64[] position
float64[] velocity
float64[] effort
bool success
```

#### Step.srv
```
# Request
float64[] effort_commands

---
# Response
float64[] position
float64[] velocity
float64[] effort
```

---

## Migration Path

### Phase 1: Preparation
1. ✅ Service message definitions exist (`Init.srv`, `Step.srv`)
2. ✅ Service clients declared in `system_interface.h`
3. ⚠️ Service clients not initialized in `system_interface.cpp`
4. ⚠️ Currently using direct singleton access

### Phase 2: Modify System Interface (C++)
```cpp
// In Simulator::on_init()
// Replace:
// m_simulation = std::thread(MuJoCoSimulator::simulate, m_mujoco_model);

// With:
m_node = std::make_shared<rclcpp::Node>("mujoco_sim_client");
m_init_client = m_node->create_client<crisp_mujoco_sim_msgs::srv::Init>("mujoco_init");
m_step_client = m_node->create_client<crisp_mujoco_sim_msgs::srv::Step>("mujoco_step");
m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
m_executor->add_node(m_node);

// Call Init service
auto init_request = std::make_shared<crisp_mujoco_sim_msgs::srv::Init::Request>();
init_request->model_path = m_mujoco_model;
auto init_future = m_init_client->async_send_request(init_request);
// Wait and process response...
```

```cpp
// In Simulator::write()
// Replace:
// MuJoCoSimulator::getInstance().write(m_effort_commands);

// With:
auto step_request = std::make_shared<crisp_mujoco_sim_msgs::srv::Step::Request>();
step_request->effort_commands = m_effort_commands;
auto step_future = m_step_client->async_send_request(step_request);
// Wait and process response, store in m_positions, m_velocities, m_efforts
```

### Phase 3: Create Python Simulator Node
```python
import rclpy
from rclpy.node import Node
import mujoco
from crisp_mujoco_sim_msgs.srv import Init, Step

class MuJoCoSimulatorNode(Node):
    def __init__(self):
        super().__init__('mujoco_simulator')
        
        self.init_srv = self.create_service(Init, 'mujoco_init', self.init_callback)
        self.step_srv = self.create_service(Step, 'mujoco_step', self.step_callback)
        
        self.model = None
        self.data = None
    
    def init_callback(self, request, response):
        self.model = mujoco.MjModel.from_xml_path(request.model_path)
        self.data = mujoco.MjData(self.model)
        # Set initial state from keyframe
        self.data.qpos[:] = self.model.keyframe(0).qpos
        
        response.position = self.data.qpos[:self.model.nu].tolist()
        response.velocity = self.data.qvel[:self.model.nu].tolist()
        response.effort = self.data.qfrc_actuator[:self.model.nu].tolist()
        response.success = True
        return response
    
    def step_callback(self, request, response):
        # Apply effort commands
        self.data.ctrl[:] = request.effort_commands
        
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        
        # Return state
        response.position = self.data.qpos[:self.model.nu].tolist()
        response.velocity = self.data.qvel[:self.model.nu].tolist()
        response.effort = self.data.qfrc_actuator[:self.model.nu].tolist()
        return response
```

### Phase 4: Launch Configuration
```python
# In launch file
simulator_node = Node(
    package='crisp_mujoco_sim',
    executable='python_mujoco_sim',
    name='mujoco_simulator',
    output='screen'
)

controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description, controller_config],
    # System Interface will connect to simulator via services
)
```

### Benefits of Migration
✅ **Language Flexibility**: Use Python MuJoCo API (often easier, better documented)  
✅ **Process Isolation**: Simulator crash doesn't kill controller manager  
✅ **Easier Debugging**: Can restart simulator independently  
✅ **Better Python Ecosystem**: Access to Python ML/visualization libraries  
✅ **Maintainability**: Cleaner separation of concerns  
⚠️ **Slight Performance Trade-off**: ~100-500μs added latency (usually acceptable for robot control)

---

## Key Differences Comparison

| Aspect | C++ In-Process | Python Service-Based |
|--------|----------------|----------------------|
| **Processes** | 1 (controller_manager) | 2 (controller_manager + python_sim) |
| **Communication** | Shared memory + mutex | ROS2 services (IPC/network) |
| **Latency** | < 1 μs | ~100-500 μs |
| **Robustness** | Tight coupling | Loose coupling |
| **Language** | C++ only | C++ interface + Python sim |
| **Debugging** | Harder (threads) | Easier (separate processes) |
| **MuJoCo API** | C API (mujoco.h) | Python API (more pythonic) |
| **Crash Behavior** | Total system crash | Isolated crash |
| **Setup Complexity** | Lower (single binary) | Higher (two nodes) |
| **Code Already Present** | ✅ Full implementation | ⚠️ Service clients declared but unused |

---

## Conclusion

Both architectures exchange the **same data** (effort commands, joint states), but differ in **how** that data is transferred:

- **Current**: High-performance in-process shared memory with mutex synchronization
- **Proposed**: Process-isolated ROS2 service-based communication with better modularity

The service-based approach (Approach 1) is **recommended** because:
1. Service definitions already exist
2. Provides clean process separation
3. Enables Python MuJoCo usage
4. Latency overhead is acceptable for most robotic applications (< 1ms)
5. Easier to debug and maintain

The System Interface layer can be **kept mostly unchanged** - only the communication mechanism needs updating from direct function calls to ROS2 service calls.
