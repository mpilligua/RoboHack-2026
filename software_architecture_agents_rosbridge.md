# Software Architecture Document

**Project:** AI-Augmented Assistive Guide Dog Robot  
**Runtime:** Laptop/PC backend communicating with robot over rosbridge  
**Agent Framework:** OpenAI Agents SDK  
**Robot Stack:** ROS/rosbridge, cameras, depth, lidar, Nav2/SLAM, person tracking, motion topics/services  
**Primary Demo Goal:** Semantic last-meter assistance: identify objects, read labels, remember them, reason over constraints, and guide the user.

---

## 1. Executive summary

This system is a laptop-side embodied-agent architecture. The laptop runs the LLM/VLM agents, semantic memory, tool registry, UI server, and rosbridge client. The robot remains the ROS capability provider: sensors, motion, person tracking, Nav2, SLAM, and low-level safety signals.

The architecture uses the OpenAI Agents SDK for agent orchestration, tool calling, handoffs, sessions, guardrails, and tracing. The Agents SDK is used for reasoning and orchestration, not for direct motor control.

Core invariant:

```text
LLM Agent → bounded tool → safety supervisor → robot client → rosbridge → ROS → robot
```

Never:

```text
LLM Agent → /cmd_vel
```

The system is organized around a semantic memory / blackboard. The fast robot/perception loop continuously updates state, objects, safety, and events. The Agents SDK layer reads this memory, reasons over goals, and calls bounded tools.

---

## 2. Design goals

### 2.1 Functional goals

The robot should be able to:

- understand user requests;
- maintain an active goal;
- observe the environment through RGB/depth/lidar;
- detect and remember objects;
- scan a shelf/table;
- read product labels;
- answer follow-up questions;
- resolve references like “it” or “the one on the left”;
- guide the user toward a selected object;
- optionally follow a person or navigate to named places.

### 2.2 Non-functional goals

The software should be:

- safe: LLMs never directly control motors;
- modular: robot, tools, memory, agents, perception, and UI are separated;
- debuggable: every tool call and state change is logged/traced;
- demo-resilient: cached scans and fallback modes exist;
- robot-agnostic: swap FakeRobot and RealRobotOverRosbridge;
- hackathon-practical: core demo works before advanced autonomy.

---

## 3. System context

```text
┌──────────────────────────────────────────────────────────────┐
│                         Phone / Web UI                        │
│  push-to-talk | transcript | plan | objects | camera | stop    │
└───────────────────────────────┬──────────────────────────────┘
                                │ HTTP / WebSocket
                                ▼
┌──────────────────────────────────────────────────────────────┐
│                         Laptop Backend                        │
│                                                              │
│  OpenAI Agents SDK                                            │
│  - Dialogue Agent                                             │
│  - Planner Agent                                              │
│  - Perception Agent                                           │
│  - Safety/Policy guardrails                                   │
│  - Tool calling                                               │
│  - Handoffs                                                   │
│  - Tracing                                                    │
│                                                              │
│  Semantic Memory / Blackboard                                 │
│  Tool Registry                                                │
│  VLM / OCR / YOLO services                                    │
│  Frame Buffer                                                 │
│  rosbridge Client                                             │
└───────────────────────────────┬──────────────────────────────┘
                                │ rosbridge websocket
                                ▼
┌──────────────────────────────────────────────────────────────┐
│                            Robot ROS                          │
│                                                              │
│  RGB camera | depth camera | lidar | person tracking          │
│  Nav2 | SLAM | odometry | TF | cmd_vel | robot status         │
└──────────────────────────────────────────────────────────────┘
```

---

## 4. Architectural layers

```text
Layer 5 — UI / Operator Layer
Layer 4 — Agents SDK Layer
Layer 3 — Tool / Skill Layer
Layer 2 — Semantic Memory / Blackboard
Layer 1 — Robot Client / rosbridge Layer
Layer 0 — ROS Robot Layer
```

---

## 5. Layer 0 — ROS robot layer

This layer runs on or with the robot. It is not responsible for LLM reasoning.

### 5.1 Responsibilities

- publish RGB frames;
- publish depth frames;
- publish lidar/pointcloud data if available;
- expose person tracking;
- expose Nav2/SLAM;
- expose odometry and TF;
- accept bounded motion commands or behavior commands;
- publish robot status and safety-relevant information.

### 5.2 Typical ROS topics

```yaml
topics:
  cmd_vel: "/cmd_vel"
  rgb: "/camera/color/image_raw"
  rgb_compressed: "/camera/color/image_raw/compressed"
  depth: "/camera/depth/image_raw"
  lidar: "/livox/lidar"
  detections: "/detections/yolo"
  person_tracking: "/person_tracking/target"
  odom: "/odom"
  tf: "/tf"
  battery: "/battery_state"
  nav2_status: "/navigate_to_pose/_action/status"
```

### 5.3 Recommended robot-side helper node

If direct Nav2/actions over rosbridge are annoying, add a simple robot-side ROS node:

```text
/assistant/behavior_command
/assistant/behavior_status
/assistant/go_to_location
/assistant/cancel_behavior
/assistant/safety_state
```

This lets the laptop send high-level commands over rosbridge rather than dealing with all ROS action details remotely.

---

## 6. Layer 1 — Robot client / rosbridge layer

This layer lives on the laptop and isolates rosbridge details from the rest of the application.

### 6.1 Responsibilities

- connect to rosbridge websocket;
- subscribe to ROS topics;
- publish ROS messages;
- call ROS services if exposed;
- wrap Nav2, camera, depth, cmd_vel, person tracking, lidar, and status;
- reconnect if rosbridge drops;
- write sensor updates into the frame buffer and memory.

### 6.2 Key modules

```text
robot/
  rosbridge_client.py
  topics.py
  ros_messages.py
  camera.py
  depth.py
  lidar.py
  cmd_vel.py
  nav2.py
  person_tracking.py
  robot_status.py
  behavior_client.py
  fake_robot.py
```

### 6.3 Dependency rule

Only files inside `robot/` should know raw rosbridge message shapes.

Good:

```python
await robot.move_forward(distance_m=0.4)
```

Bad:

```python
await ros.publish("/cmd_vel", "geometry_msgs/Twist", {...})
```

inside agents or high-level tools.

---

## 7. Layer 2 — Semantic memory / blackboard

The memory layer is the center of the system.

Agents, tools, perception workers, and robot clients communicate through memory and events instead of directly calling each other.

### 7.1 Responsibilities

- store robot state;
- store object memory;
- store user/session state;
- store active goals;
- store known places;
- store recent events;
- store current plan;
- store conversation state or reference to SDK session;
- provide compact snapshots to agents.

### 7.2 Memory modules

```text
memory/
  store.py
  schemas.py
  object_registry.py
  goal_manager.py
  place_memory.py
  event_log.py
  snapshots.py
```

### 7.3 Core schemas

#### Robot state

```python
@dataclass
class RobotState:
    mode: str = "idle"
    current_behavior: str | None = None

    path_clear: bool = True
    safety_stop: bool = False
    nearest_obstacle_m: float | None = None

    person_visible: bool = False
    person_distance_m: float | None = None
    person_bearing_deg: float | None = None

    nav2_status: str | None = None
    nav_goal: str | None = None

    latest_rgb_frame_id: str | None = None
    latest_depth_frame_id: str | None = None

    battery_percent: float | None = None
```

#### Object memory

```python
@dataclass
class ObjectMemory:
    id: str
    label: str
    confidence: float

    last_seen_ts: float
    seen_count: int

    bbox: tuple[int, int, int, int] | None = None
    mask_id: str | None = None
    depth_m: float | None = None
    bearing_deg: float | None = None

    position_text: str | None = None
    place: str | None = None

    visible_text: list[str] = field(default_factory=list)
    attributes: dict = field(default_factory=dict)
    risks: list[str] = field(default_factory=list)

    sources: list[str] = field(default_factory=list)
    status: str = "visible"  # visible, occluded, stale, removed
```

#### Goal

```python
@dataclass
class Goal:
    id: str
    description: str
    status: str              # pending, active, blocked, complete, failed, cancelled
    priority: int
    created_by: str          # user, planner, safety, operator
    success_condition: str
    constraints: dict
    current_step: str | None = None
    selected_object_id: str | None = None
```

#### Event

```python
@dataclass
class Event:
    timestamp: float
    type: str
    source: str
    message: str
    data: dict | None = None
```

### 7.4 Memory snapshot for agents

The Agents SDK should receive compact memory snapshots, not raw frame buffers or full logs.

```python
def snapshot_for_agent(memory: MemoryStore) -> dict:
    return {
        "robot": memory.robot.to_public_dict(),
        "active_goal": memory.goals.active_public_dict(),
        "objects": [obj.to_public_dict() for obj in memory.objects.visible_or_recent()],
        "places": memory.places.public_dict(),
        "recent_events": memory.events.last_n(20),
        "current_plan": memory.current_plan,
    }
```

---

## 8. Layer 3 — Tool / skill layer

The tool layer exposes bounded robot capabilities to agents.

### 8.1 Responsibilities

- implement all robot tools;
- validate access control;
- invoke safety checks;
- call robot clients;
- update memory;
- emit events;
- return structured results.

### 8.2 Tool groups

```text
tools/
  base.py
  registry.py
  safety_tools.py
  motion_tools.py
  pose_tools.py
  sensor_tools.py
  perception_tools.py
  memory_tools.py
  autonomy_tools.py
  interaction_tools.py
  operator_tools.py
```

### 8.3 Tool interface

```python
@dataclass
class ToolResult:
    ok: bool
    tool: str
    result: dict | None = None
    error: str | None = None
    events: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

class Tool:
    name: str
    tier: int
    description: str
    allowed_callers: list[str]
    requires_safety_check: bool = False

    async def run(self, args: dict, context: "ToolContext") -> ToolResult:
        raise NotImplementedError
```

### 8.4 Tool context

```python
@dataclass
class ToolContext:
    memory: MemoryStore
    robot: RobotClient
    safety: SafetySupervisor
    event_stream: EventStream
    frame_buffer: FrameBuffer
    perception: PerceptionServices
```

### 8.5 Tool registry

```python
class ToolRegistry:
    def __init__(self):
        self.tools: dict[str, Tool] = {}

    def register(self, tool: Tool):
        self.tools[tool.name] = tool

    async def call(self, name: str, args: dict, context: ToolContext, caller: str) -> ToolResult:
        tool = self.tools[name]

        if caller not in tool.allowed_callers:
            return ToolResult(
                ok=False,
                tool=name,
                error=f"{caller} is not allowed to call {name}",
                events=[f"Tool call blocked: {name}"],
            )

        if tool.requires_safety_check:
            safety = await context.safety.check_safety()
            if not safety.safe_to_move:
                return ToolResult(
                    ok=False,
                    tool=name,
                    error=f"Safety check failed: {safety.reason}",
                    events=[f"Safety blocked tool: {name}"],
                )

        result = await tool.run(args, context)
        context.memory.events.add_tool_result(result)
        await context.event_stream.publish_tool_result(result)
        return result
```

---

## 9. Layer 4 — Agents SDK layer

This layer uses the OpenAI Agents SDK for LLM orchestration.

### 9.1 Why use the Agents SDK here

Use the SDK because the architecture naturally maps to:

- agents with specialized instructions;
- function tools for robot capabilities;
- handoffs between specialized agents;
- sessions for multi-turn conversation;
- guardrails for safety/policy checks;
- tracing for debugging generations, tool calls, handoffs, and guardrails.

### 9.2 Agent roles

Use three main agents and one deterministic safety supervisor.

```text
Dialogue Agent
- understands user utterances
- extracts constraints
- creates or updates goals
- produces user-facing wording

Planner Agent
- reads memory snapshot
- chooses next skill/tool
- updates plan
- recovers from failures

Perception Agent
- uses VLM/OCR/object detection tools
- enriches object memory
- verifies labels and claims

Safety Supervisor
- deterministic/local policy layer
- blocks unsafe movement
- validates tool calls
- can trigger emergency stop
```

### 9.3 Recommended orchestration style

Use a **manager-style orchestrator** with optional handoffs.

```text
User message
→ Orchestrator
→ Dialogue Agent
→ Planner Agent
→ Perception Agent if needed
→ Tool call
→ Memory update
→ UI update
→ Speak response
```

Do not let agents freely talk to each other in loops. Prefer:

```text
All agents read memory.
All agents write structured updates.
The orchestrator manages handoffs and tool execution.
```

### 9.4 Agents SDK components

#### Agent definitions

```python
from agents import Agent

dialogue_agent = Agent(
    name="Dialogue Agent",
    instructions=load_prompt("prompts/dialogue.md"),
    tools=[
        set_active_goal_tool,
        get_active_goal_tool,
        speak_tool,
        display_plan_tool,
    ],
    handoffs=[planner_agent],
)

planner_agent = Agent(
    name="Planner Agent",
    instructions=load_prompt("prompts/planner.md"),
    tools=[
        get_robot_status_tool,
        check_safety_tool,
        get_visible_objects_tool,
        find_object_tool,
        resolve_reference_tool,
        find_objects_matching_constraints_tool,
        scan_shelf_tool,
        read_label_tool,
        go_to_location_tool,
        follow_person_tool,
        turn_toward_object_tool,
        approach_object_tool,
        speak_tool,
        stop_tool,
    ],
    handoffs=[perception_agent],
)

perception_agent = Agent(
    name="Perception Agent",
    instructions=load_prompt("prompts/perception.md"),
    tools=[
        get_rgb_frame_tool,
        get_depth_frame_tool,
        get_depth_at_pixel_tool,
        detect_objects_tool,
        segment_scene_tool,
        read_text_from_image_tool,
        ask_vlm_about_scene_tool,
        ask_vlm_about_object_tool,
        remember_object_tool,
        refresh_object_position_tool,
    ],
)
```

#### Runner use

```python
from agents import Runner

result = await Runner.run(
    dialogue_agent,
    input=user_message,
    session=session,
    context=agent_context,
)
```

### 9.5 Agent context

Agents need access to memory and tool context.

```python
@dataclass
class AgentContext:
    memory: MemoryStore
    tools: ToolRegistry
    tool_context: ToolContext
    event_stream: EventStream
    session_id: str
```

Agents should not receive robot clients directly.

---

## 10. Agents SDK tool wrappers

The SDK-facing tools should be thin wrappers around the internal tool registry.

### 10.1 Example wrapper

```python
from agents import function_tool

@function_tool
async def scan_shelf(ctx, motion: bool = True, target: str = "front_shelf") -> dict:
    result = await ctx.context.tools.call(
        name="scan_shelf",
        args={"motion": motion, "target": target},
        context=ctx.context.tool_context,
        caller="planner",
    )
    return result.__dict__
```

### 10.2 Safety-sensitive wrapper

```python
@function_tool
async def approach_object(ctx, object_id: str, stop_distance_m: float = 0.5) -> dict:
    result = await ctx.context.tools.call(
        name="approach_object",
        args={"object_id": object_id, "stop_distance_m": stop_distance_m},
        context=ctx.context.tool_context,
        caller="planner",
    )
    return result.__dict__
```

### 10.3 Why wrappers should be thin

The internal tool registry remains the source of truth for:

- allowed callers;
- safety checks;
- logging;
- memory updates;
- fallback behavior.

The Agents SDK tool functions only adapt SDK tool calls into the internal tool API.

---

## 11. Guardrails and approvals

### 11.1 Input guardrails

Use input guardrails to catch dangerous user instructions before planning.

Examples:

```text
"Run fast into the wall"
"Ignore safety"
"Disable stop"
```

### 11.2 Tool guardrails

Movement tools should be guarded by deterministic checks:

```text
safety_stop == false
path_clear == true
nearest_obstacle_m >= threshold
robot_status is fresh
command distance/speed within limits
```

### 11.3 Human-in-the-loop approvals

For the hackathon, require operator approval for:

```text
emergency reset
raw Nav2 goal
map save/load if risky
switch_to_real_robot
large movement command
```

Normal demo tools should not require manual approval, or the demo will feel slow.

---

## 12. Sessions and conversation memory

Use Agents SDK sessions for conversation continuity, and use `MemoryStore` for robot/world state.

### 12.1 Distinction

```text
SDK session memory:
- user-agent conversation history
- prior tool-call context
- dialogue continuity

Semantic memory:
- robot state
- objects
- places
- goals
- recent events
- object locations
- safety state
```

Do not rely on conversation history alone to remember objects. Object memory should be explicit and structured.

---

## 13. Tracing and observability

Use Agents SDK tracing for:

- LLM generations;
- tool calls;
- handoffs;
- guardrails;
- custom spans around robot behaviors.

Also log local events in `memory.event_log`.

### 13.1 Custom trace spans

Create custom spans for:

```text
rosbridge_connect
rgb_frame_received
scan_shelf
read_label
nav2_goal
follow_person
approach_object
safety_block
cached_scan_used
```

### 13.2 Event stream

Every significant event should also go to the UI:

```json
{
  "type": "tool_started",
  "source": "planner",
  "message": "Scanning shelf",
  "data": {"tool": "scan_shelf"}
}
```

---

## 14. Layer 5 — UI / operator layer

The UI is not just decoration. It is the live explanation interface.

### 14.1 UI responsibilities

- push-to-talk or text input;
- transcript;
- current robot mode;
- active goal;
- current plan;
- detected objects;
- latest camera frame preview;
- safety status;
- stop/emergency stop button;
- operator fallback controls.

### 14.2 Recommended routes

```text
GET  /health
GET  /memory
GET  /objects
GET  /goals
GET  /robot/status

POST /user/message
POST /tools/{tool_name}
POST /operator/stop
POST /operator/emergency-stop
POST /operator/use-cached-scan
POST /operator/fallback-stationary

WS   /ws/events
```

### 14.3 WebSocket event types

```text
robot_state_update
memory_update
plan_update
tool_started
tool_finished
speech
error
camera_frame_reference
safety_update
agent_trace_summary
```

---

## 15. Repo structure

```text
robot-guide-assistant/
├── README.md
├── pyproject.toml
├── .env.example
├── docker-compose.yml
│
├── configs/
│   ├── default.yaml
│   ├── robot_lite3.yaml
│   ├── demo.yaml
│   ├── topics.yaml
│   └── tools.yaml
│
├── backend/
│   ├── main.py
│   ├── app.py
│   ├── dependencies.py
│   └── logging_config.py
│
├── agents_app/
│   ├── __init__.py
│   ├── orchestrator.py
│   ├── sdk_agents.py
│   ├── sdk_tools.py
│   ├── sessions.py
│   ├── guardrails.py
│   ├── tracing.py
│   └── prompts/
│       ├── dialogue.md
│       ├── planner.md
│       ├── perception.md
│       └── system.md
│
├── memory/
│   ├── __init__.py
│   ├── store.py
│   ├── schemas.py
│   ├── object_registry.py
│   ├── goal_manager.py
│   ├── place_memory.py
│   ├── event_log.py
│   └── snapshots.py
│
├── tools/
│   ├── __init__.py
│   ├── base.py
│   ├── registry.py
│   ├── safety_tools.py
│   ├── motion_tools.py
│   ├── pose_tools.py
│   ├── sensor_tools.py
│   ├── perception_tools.py
│   ├── memory_tools.py
│   ├── autonomy_tools.py
│   ├── interaction_tools.py
│   └── operator_tools.py
│
├── robot/
│   ├── __init__.py
│   ├── rosbridge_client.py
│   ├── ros_messages.py
│   ├── topics.py
│   ├── cmd_vel.py
│   ├── camera.py
│   ├── depth.py
│   ├── lidar.py
│   ├── nav2.py
│   ├── person_tracking.py
│   ├── robot_status.py
│   ├── behavior_client.py
│   └── fake_robot.py
│
├── perception/
│   ├── __init__.py
│   ├── frame_buffer.py
│   ├── depth_utils.py
│   ├── object_detector.py
│   ├── segmentation.py
│   ├── ocr.py
│   ├── vlm_client.py
│   ├── shelf_scanner.py
│   └── spatial_grounding.py
│
├── skills/
│   ├── __init__.py
│   ├── scan_shelf.py
│   ├── read_label.py
│   ├── approach_object.py
│   ├── follow_person.py
│   ├── go_to_location.py
│   └── describe_location.py
│
├── ui/
│   ├── websocket.py
│   ├── event_stream.py
│   ├── api_routes.py
│   └── schemas.py
│
├── frontend/
│   └── phone-ui/
│       ├── package.json
│       ├── index.html
│       └── src/
│           ├── App.tsx
│           ├── components/
│           │   ├── Transcript.tsx
│           │   ├── RobotStatus.tsx
│           │   ├── PlanView.tsx
│           │   ├── ObjectMemoryView.tsx
│           │   ├── CameraView.tsx
│           │   └── OperatorPanel.tsx
│           └── api/
│               └── websocket.ts
│
├── scripts/
│   ├── run_backend.sh
│   ├── run_frontend.sh
│   ├── smoke_test_rosbridge.py
│   ├── check_topics.py
│   ├── save_demo_scan.py
│   ├── replay_demo_scan.py
│   └── run_demo.py
│
├── tests/
│   ├── test_memory.py
│   ├── test_tool_registry.py
│   ├── test_reference_resolution.py
│   ├── test_fake_robot.py
│   ├── test_safety_supervisor.py
│   └── test_sdk_tools.py
│
└── data/
    ├── cached_scans/
    ├── frames/
    ├── logs/
    └── maps/
```

---

## 16. Main backend startup flow

When `backend.main` starts:

```text
1. Load configuration.
2. Create MemoryStore.
3. Create EventStream.
4. Create FrameBuffer.
5. Connect to rosbridge.
6. Start robot subscriptions.
7. Create RobotClient wrappers.
8. Create SafetySupervisor.
9. Register internal tools.
10. Wrap internal tools as Agents SDK function tools.
11. Create Agents SDK agents.
12. Create session manager.
13. Start FastAPI and WebSocket server.
14. Run self-check.
15. Wait for user/operator input.
```

Pseudo-code:

```python
async def main():
    config = load_config()

    memory = MemoryStore()
    event_stream = EventStream()
    frame_buffer = FrameBuffer(max_frames=config.frames.max_frames)

    ros = RosbridgeClient(config.rosbridge.url)
    await ros.connect()

    robot = RobotClient(
        ros=ros,
        frame_buffer=frame_buffer,
        config=config,
        memory=memory,
        event_stream=event_stream,
    )

    await robot.start_subscriptions()

    safety = SafetySupervisor(
        memory=memory,
        robot=robot,
        config=config.safety,
    )

    tool_context = ToolContext(
        memory=memory,
        robot=robot,
        safety=safety,
        event_stream=event_stream,
        frame_buffer=frame_buffer,
        perception=build_perception_services(config),
    )

    registry = build_internal_tool_registry()
    sdk_tools = build_agents_sdk_tools(registry, tool_context)

    agents = build_agents(sdk_tools=sdk_tools, config=config)
    sessions = SessionManager(config=config)

    orchestrator = AgentOrchestrator(
        agents=agents,
        sessions=sessions,
        memory=memory,
        event_stream=event_stream,
        tool_context=tool_context,
    )

    app = create_app(
        memory=memory,
        event_stream=event_stream,
        orchestrator=orchestrator,
        tool_registry=registry,
    )

    await run_fastapi(app, host=config.server.host, port=config.server.port)
```

---

## 17. Agent orchestration flow

### 17.1 User asks: “Find me a healthy snack without nuts.”

```text
UI
→ POST /user/message
→ Orchestrator
→ Dialogue Agent
→ set_active_goal()
→ Planner Agent
→ display_plan()
→ scan_shelf()
→ Perception Agent handoff if needed
→ read_label()
→ find_objects_matching_constraints()
→ speak()
→ Memory update
→ UI event stream
```

### 17.2 Follow-up: “Take me to it.”

```text
UI
→ POST /user/message
→ Dialogue Agent resolves reference intent
→ Planner Agent calls resolve_reference("it")
→ check_safety()
→ turn_toward_object()
→ approach_object() if safe
→ speak("The apple is in front of you, slightly left.")
```

---

## 18. Config files

### 18.1 `configs/topics.yaml`

```yaml
rosbridge:
  url: "ws://192.168.1.42:9090"

topics:
  cmd_vel: "/cmd_vel"
  rgb: "/camera/color/image_raw/compressed"
  depth: "/camera/depth/image_raw"
  detections: "/detections/yolo"
  person_tracking: "/person_tracking/target"
  odom: "/odom"
  tf: "/tf"
  battery: "/battery_state"
  nav2_status: "/navigate_to_pose/_action/status"

frames:
  base: "base_link"
  map: "map"
  camera: "camera_color_optical_frame"
```

### 18.2 `configs/tools.yaml`

```yaml
safety:
  max_forward_speed_mps: 0.25
  max_lateral_speed_mps: 0.15
  max_motion_distance_m: 1.0
  obstacle_stop_distance_m: 0.4
  command_timeout_s: 5.0
  require_fresh_status_s: 1.0

planner_exposed_tools:
  - get_robot_status
  - check_safety
  - speak
  - display_plan
  - get_visible_objects
  - find_object
  - resolve_reference
  - find_objects_matching_constraints
  - describe_scene
  - scan_shelf
  - read_label
  - follow_person
  - go_to_location
  - turn_toward_object
  - approach_object
  - stop

operator_only_tools:
  - emergency_stop
  - reset_safety_latch
  - send_nav2_goal
  - switch_to_real_robot
  - switch_to_fake_robot
```

### 18.3 `configs/demo.yaml`

```yaml
demo:
  locations:
    home:
      x: 0.0
      y: 0.0
      yaw: 0.0
      description: "Starting position"

    shelf:
      x: 1.8
      y: 0.4
      yaw: 0.0
      description: "Low product display table"

  cached_scan_name: "good_shelf_scan"

  fallback:
    allow_cached_scan: true
    allow_stationary_mode: true
    allow_text_mode: true
```

---

## 19. Core tool subset for MVP

Implement these first:

```python
# Safety
stop()
emergency_stop()
check_safety()
get_robot_status()

# Interaction
speak()
display_plan()

# Sensors/perception
get_rgb_frame()
get_depth_frame()
get_depth_at_pixel()
detect_objects()
describe_scene()
scan_shelf()
read_label()

# Memory
get_visible_objects()
find_object()
resolve_reference()
find_objects_matching_constraints()

# Motion/autonomy
move_forward()
rotate()
turn_toward_object()
follow_person()       # if existing person tracking works
go_to_location()      # if Nav2 works
```

---

## 20. Recommended first implementation milestone

Build a working vertical slice:

```text
User text input
→ Agents SDK Dialogue Agent
→ Planner Agent
→ scan_shelf tool
→ latest camera frame through rosbridge
→ VLM shelf scan
→ object memory update
→ speak response
→ UI displays objects and plan
```

This vertical slice proves the whole architecture:

```text
UI → Agent → Tool → Robot sensor → VLM → Memory → UI/Speech
```

Movement can be added after this works.

---

## 21. Testing strategy

### 21.1 Unit tests

```text
test_memory.py
- object insertion
- object update
- stale object handling
- reference resolution

test_tool_registry.py
- allowed caller
- denied caller
- safety-gated tool

test_safety_supervisor.py
- obstacle too close
- stale robot status
- emergency stop latch

test_sdk_tools.py
- SDK wrapper calls internal registry
- tool result shape
```

### 21.2 Integration tests

```text
smoke_test_rosbridge.py
- connect to rosbridge
- subscribe to status
- get one camera frame
- publish stop

run_demo_scan.py
- get frame
- scan shelf
- update memory
- save cached scan
```

### 21.3 Demo tests

```text
1. Stationary shelf scan
2. Label read
3. Allergen question
4. Follow-up reference: “take me to it”
5. Cached scan fallback
6. Emergency stop
```

---

## 22. Failure and fallback behavior

### 22.1 Camera failure

Fallback:

```text
use_cached_scan()
speak("I’m using my previous scan because the camera feed is unavailable.")
```

### 22.2 VLM failure

Fallback:

```text
use YOLO/object memory
speak("I can identify the objects, but I cannot reliably read the label right now.")
```

### 22.3 Movement failure

Fallback:

```text
fallback_to_stationary_mode()
continue semantic shelf assistant demo
```

### 22.4 Voice/TTS failure

Fallback:

```text
fallback_to_text_mode()
show responses on phone UI
```

---

## 23. Security and safety boundaries

### 23.1 LLM restrictions

The planner must not be able to:

- call raw `/cmd_vel`;
- disable safety;
- reset emergency stop;
- send arbitrary Nav2 poses unless operator-approved;
- execute shell commands;
- modify ROS config.

### 23.2 Tool access control

| Tool group | Planner | Perception Agent | Safety Supervisor | Operator |
|---|---:|---:|---:|---:|
| Speak/display | Yes | No | Yes | Yes |
| RGB/depth | Limited | Yes | No | Yes |
| VLM/OCR | Limited | Yes | No | Yes |
| Movement | Limited | No | Validate | Yes |
| Emergency stop | Yes | No | Yes | Yes |
| Reset safety | No | No | No | Yes |
| Raw Nav2 goal | No | No | No | Yes |
| Cached scan/replay | Yes | Yes | No | Yes |
| Fake/real switch | No | No | No | Yes |

---

## 24. Deployment model

### 24.1 Laptop

Runs:

```text
backend FastAPI server
Agents SDK runtime
VLM/LLM clients
Frame buffer
Semantic memory
Tool registry
UI WebSocket
optional local YOLO/segmentation
```

### 24.2 Robot

Runs:

```text
rosbridge_server
camera/depth/lidar publishers
person tracking
Nav2/SLAM
motion stack
optional YOLO/segmentation
```

### 24.3 Network

```text
Laptop and robot on same Wi-Fi/LAN.
Laptop connects to ws://ROBOT_IP:9090.
Phone connects to laptop backend/frontend.
```

---

## 25. Development order

```text
1. rosbridge smoke test
2. FastAPI backend + event stream
3. MemoryStore + schemas
4. FrameBuffer + camera subscription
5. Agents SDK minimal Dialogue/Planner agent
6. Tool registry + SDK wrappers
7. speak/display_plan/get_robot_status/check_safety
8. scan_shelf with latest frame
9. object memory + reference resolution
10. read_label
11. motion tools: stop, rotate, move_forward
12. turn_toward_object
13. follow_person or go_to_location
14. cached scan fallback
15. UI polish and demo script
```

---

## 26. Minimal viable architecture

If time is short, build this:

```text
backend/
  main.py
agents_app/
  sdk_agents.py
  sdk_tools.py
memory/
  store.py
  schemas.py
tools/
  registry.py
  safety_tools.py
  perception_tools.py
  memory_tools.py
  interaction_tools.py
robot/
  rosbridge_client.py
  camera.py
  cmd_vel.py
perception/
  frame_buffer.py
  vlm_client.py
ui/
  websocket.py
configs/
  topics.yaml
scripts/
  smoke_test_rosbridge.py
```

Minimum working demo:

```text
User: "What's on the shelf?"
→ scan_shelf()
→ object memory
→ speak()

User: "Anything with nuts?"
→ read_label()
→ speak()

User: "Which is healthiest?"
→ find_objects_matching_constraints()
→ speak()

User: "Take me to it."
→ resolve_reference()
→ turn_toward_object()
→ speak()
```

---

## 27. Final architecture statement

The system is a laptop-side embodied agent runtime using the OpenAI Agents SDK. The SDK manages dialogue, planning, perception handoffs, sessions, tool calls, guardrails, and tracing. A semantic memory layer stores the robot’s world model: objects, goals, places, robot state, and events. Tools expose bounded robot capabilities. The robot is accessed through rosbridge, wrapped by robot clients and safety-validated skills. The LLM agents reason over memory and call tools, but the local robot layer owns execution and safety.

The project’s technical claim is:

```text
Fast ROS control and perception provide embodied state.
Semantic memory provides continuity.
Agents provide reasoning and dialogue.
Bounded tools connect reasoning to robot action safely.
```
