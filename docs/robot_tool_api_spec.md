# Robot Tool API Specification

**Project:** AI-Augmented Assistive Guide Dog Robot  
**Purpose:** Define the robot-facing and agent-facing tool surface for a memory-centric, dual-loop, multi-agent assistive robotics system.

---

## 1. Architectural principle

The robot system is organized around a simple invariant:

> The LLM/planner calls bounded tools.  
> The local ROS/control layer validates and executes them.  
> The LLM must never publish raw motor commands directly.

Recommended execution chain:

```text
Agent proposes action
→ Semantic memory records proposed action
→ Safety supervisor validates action
→ Skill/tool executes through ROS/local controller
→ Skill result updates memory
→ UI displays state, plan, and result
```

Never:

```text
LLM → /cmd_vel
```

Always:

```text
LLM → tool/skill → behavior controller → ROS → robot
```

---

## 2. Tool tier system

Tools are divided by how indispensable they are.

```text
Tier 0 — Safety-critical / always available
Tier 1 — MVP demo-critical
Tier 2 — Strong demo / high-value autonomy
Tier 3 — Advanced architecture / stretch
Tier 4 — Operator, debugging, and fallback tools
```

### Tier meanings

| Tier | Meaning | Build priority |
|---|---|---|
| Tier 0 | Required before any autonomy or movement | Must build first |
| Tier 1 | Required for the core shelf-assistant demo | Build for MVP |
| Tier 2 | Makes the demo feel embodied and autonomous | Build after MVP works |
| Tier 3 | Advanced technical depth and assistive UX | Stretch |
| Tier 4 | Demo rescue, diagnostics, operator control | Build selectively but early enough to save demos |

---

## 3. Standard tool result schema

Every tool should return the same top-level structure.

```python
ToolResult = {
    "ok": bool,
    "tool": str,
    "result": dict | None,
    "error": str | None,
    "events": list[str],
    "timestamp": float
}
```

For movement tools, include safety metadata:

```python
{
    "safety": {
        "validated": bool,
        "stopped_due_to_obstacle": bool,
        "nearest_obstacle_m": float | None
    }
}
```

Recommended example:

```json
{
  "ok": true,
  "tool": "move_forward",
  "result": {
    "requested_distance_m": 0.5,
    "estimated_distance_m": 0.48,
    "stopped_due_to_obstacle": false
  },
  "error": null,
  "events": ["Moved forward 0.48 meters"],
  "timestamp": 1730001234.12,
  "safety": {
    "validated": true,
    "stopped_due_to_obstacle": false,
    "nearest_obstacle_m": 0.82
  }
}
```

---

## 4. Tier 0 — Safety-critical tools

These tools must exist before autonomous movement runs.

---

### 4.1 `stop`

Immediately stop robot motion.

```python
stop() -> ToolResult
```

#### Behavior

- Publish zero velocity.
- Stop currently executing low-level motion.
- May leave higher-level task active unless explicitly cancelled.
- Must be callable by:
  - planner
  - UI
  - operator
  - direct voice interrupt
  - safety supervisor

#### Example return

```json
{
  "ok": true,
  "tool": "stop",
  "result": {
    "robot_mode": "stopped"
  },
  "error": null,
  "events": ["Robot stopped"],
  "timestamp": 1730001234.12
}
```

---

### 4.2 `emergency_stop`

Hard stop and safety latch.

```python
emergency_stop(reason: str | None = None) -> ToolResult
```

#### Behavior

- Publish zero velocity immediately.
- Cancel current behavior.
- Cancel Nav2 goal.
- Set safety latch.
- Requires manual/operator reset.

#### Example return

```json
{
  "ok": true,
  "tool": "emergency_stop",
  "result": {
    "safety_latched": true,
    "reason": "Obstacle too close"
  },
  "error": null,
  "events": ["Emergency stop activated"],
  "timestamp": 1730001234.12
}
```

---

### 4.3 `check_safety`

Return whether the robot is allowed to move.

```python
check_safety() -> ToolResult
```

#### Return fields

```json
{
  "safe_to_move": true,
  "path_clear": true,
  "nearest_obstacle_m": 0.82,
  "safety_stop": false,
  "person_nearby": true,
  "reason": null
}
```

---

### 4.4 `get_robot_status`

Return compact robot state.

```python
get_robot_status() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "get_robot_status",
  "result": {
    "mode": "idle",
    "current_behavior": null,
    "battery_percent": 78,
    "nav2_status": "inactive",
    "person_visible": true,
    "path_clear": true,
    "nearest_obstacle_m": 1.1
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.12
}
```

---

### 4.5 `cancel_current_action`

Cancel the active semantic behavior or skill.

```python
cancel_current_action(reason: str | None = None) -> ToolResult
```

#### Difference from `stop`

- `stop()` stops motion.
- `cancel_current_action()` cancels the active high-level behavior such as following, scanning, or navigating.

---

### 4.6 `reset_safety_latch`

Reset safety latch after emergency stop.

```python
reset_safety_latch(operator_confirmed: bool) -> ToolResult
```

#### Access control

This should be operator-only, not exposed to the main planner.

---

## 5. Tier 1 — MVP demo-critical tools

These are required for the core shelf-assistant demo.

---

### 5.1 `speak`

Speak to the user.

```python
speak(text: str, priority: str = "normal") -> ToolResult
```

#### Priority values

```text
normal
urgent
safety
```

#### Example call

```json
{
  "text": "I see an apple, a granola bar, and orange juice.",
  "priority": "normal"
}
```

#### Example return

```json
{
  "ok": true,
  "tool": "speak",
  "result": {
    "spoken": true,
    "text": "I see an apple, a granola bar, and orange juice."
  },
  "error": null,
  "events": ["Spoke response"],
  "timestamp": 1730001234.12
}
```

---

### 5.2 `display_plan`

Send the current plan to the phone/UI.

```python
display_plan(plan: list[str], status: str = "thinking") -> ToolResult
```

#### Example call

```json
{
  "plan": [
    "Scan the shelf",
    "Identify food items",
    "Check labels for nut risk",
    "Recommend the safest healthy option"
  ],
  "status": "thinking"
}
```

---

### 5.3 `get_rgb_frame`

Get the latest RGB frame.

```python
get_rgb_frame(camera: str = "front") -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "get_rgb_frame",
  "result": {
    "frame_id": "rgb_1730001234",
    "camera": "front",
    "width": 1280,
    "height": 720,
    "timestamp": 1730001234.12
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.13
}
```

The actual image should be stored in a frame buffer and referenced by `frame_id`.

---

### 5.4 `get_depth_frame`

Get the latest depth frame.

```python
get_depth_frame(camera: str = "front") -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "get_depth_frame",
  "result": {
    "frame_id": "depth_1730001234",
    "camera": "front",
    "width": 640,
    "height": 480,
    "timestamp": 1730001234.13
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.14
}
```

---

### 5.5 `get_depth_at_pixel`

Get median depth at a pixel.

```python
get_depth_at_pixel(
    x: int,
    y: int,
    camera: str = "front",
    window_px: int = 5
) -> ToolResult
```

#### Notes

Use a small median window. A single depth pixel is often noisy.

#### Example return

```json
{
  "ok": true,
  "tool": "get_depth_at_pixel",
  "result": {
    "x": 420,
    "y": 260,
    "depth_m": 0.82,
    "window_px": 5,
    "valid_ratio": 0.91
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.12
}
```

---

### 5.6 `detect_objects`

Run object detection on the latest or selected frame.

```python
detect_objects(
    frame_id: str | None = None,
    classes: list[str] | None = None,
    confidence_threshold: float = 0.4
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "detect_objects",
  "result": {
    "frame_id": "rgb_1730001234",
    "detections": [
      {
        "detection_id": "det_001",
        "label": "apple",
        "confidence": 0.91,
        "bbox": [340, 210, 430, 320],
        "source": "yolo"
      },
      {
        "detection_id": "det_002",
        "label": "bottle",
        "confidence": 0.82,
        "bbox": [520, 160, 650, 430],
        "source": "yolo"
      }
    ]
  },
  "error": null,
  "events": ["Detected 2 objects"],
  "timestamp": 1730001234.12
}
```

---

### 5.7 `describe_scene`

Use VLM or local perception to describe the scene.

```python
describe_scene(
    frame_id: str | None = None,
    detail_level: str = "brief"
) -> ToolResult
```

#### Detail levels

```text
brief
normal
detailed
assistive
```

#### Example return

```json
{
  "ok": true,
  "tool": "describe_scene",
  "result": {
    "description": "There is a low table in front of the robot with several food items, including an apple, a granola bar, and a bottle.",
    "notable_objects": ["apple", "granola bar", "bottle"],
    "uncertainties": []
  },
  "error": null,
  "events": ["Scene described"],
  "timestamp": 1730001234.12
}
```

---

### 5.8 `scan_shelf`

Core demo tool. Scans a visible shelf/table and creates object memory.

```python
scan_shelf(
    motion: bool = True,
    target: str = "front_shelf"
) -> ToolResult
```

#### Behavior

- Optionally performs slow scan motion.
- Captures several frames.
- Runs object detection.
- Optionally runs VLM.
- Creates or updates persistent object IDs.
- Updates semantic memory.

#### Example return

```json
{
  "ok": true,
  "tool": "scan_shelf",
  "result": {
    "scan_id": "scan_001",
    "items": [
      {
        "object_id": "obj_apple_1",
        "name": "apple",
        "position": "center-left",
        "bearing_deg": -8.0,
        "depth_m": 0.72,
        "confidence": 0.93,
        "attributes": {
          "category": "food",
          "healthy_candidate": true,
          "contains_nuts": false
        },
        "visible_text": []
      },
      {
        "object_id": "obj_granola_1",
        "name": "granola bar",
        "position": "left",
        "bearing_deg": -18.0,
        "depth_m": 0.76,
        "confidence": 0.86,
        "attributes": {
          "category": "snack",
          "contains_nuts": true
        },
        "visible_text": ["almond", "protein"],
        "risks": ["contains almonds"]
      }
    ]
  },
  "error": null,
  "events": [
    "Scanned shelf",
    "Found apple and granola bar"
  ],
  "timestamp": 1730001234.12
}
```

---

### 5.9 `read_label`

Read visible label/text on an object.

```python
read_label(object_id: str, question: str | None = None) -> ToolResult
```

#### Example call

```json
{
  "object_id": "obj_granola_1",
  "question": "Does this contain nuts?"
}
```

#### Example return

```json
{
  "ok": true,
  "tool": "read_label",
  "result": {
    "object_id": "obj_granola_1",
    "visible_text": ["almond", "protein", "may contain traces of nuts"],
    "answer": "Yes. The visible text suggests it contains almonds and may contain nuts.",
    "confidence": 0.84,
    "risks": ["contains almonds", "may contain nuts"]
  },
  "error": null,
  "events": ["Read label for obj_granola_1"],
  "timestamp": 1730001234.12
}
```

---

### 5.10 `find_object`

Find an object from memory or current perception.

```python
find_object(query: str) -> ToolResult
```

#### Example call

```json
{
  "query": "healthy snack without nuts"
}
```

#### Example return

```json
{
  "ok": true,
  "tool": "find_object",
  "result": {
    "matches": [
      {
        "object_id": "obj_apple_1",
        "label": "apple",
        "score": 0.92,
        "reason": "Whole fruit, no nut risk, visible on shelf"
      }
    ]
  },
  "error": null,
  "events": ["Found matching object: apple"],
  "timestamp": 1730001234.12
}
```

---

### 5.11 `resolve_reference`

Resolve natural language references such as “it,” “the left one,” or “the healthy one.”

```python
resolve_reference(reference: str) -> ToolResult
```

#### Example call

```json
{
  "reference": "it"
}
```

#### Example return

```json
{
  "ok": true,
  "tool": "resolve_reference",
  "result": {
    "object_id": "obj_apple_1",
    "label": "apple",
    "confidence": 0.89,
    "reason": "The apple was the most recently recommended object."
  },
  "error": null,
  "events": ["Resolved reference 'it' to obj_apple_1"],
  "timestamp": 1730001234.12
}
```

---

### 5.12 `move_forward`

Bounded forward movement.

```python
move_forward(
    distance_m: float,
    speed_mps: float = 0.15
) -> ToolResult
```

#### Recommended safety limits

```text
max distance per command: 1.0 m
max speed: 0.25 m/s
requires check_safety()
timeout required
stop if obstacle < 0.4 m
```

#### Example return

```json
{
  "ok": true,
  "tool": "move_forward",
  "result": {
    "requested_distance_m": 0.5,
    "estimated_distance_m": 0.48,
    "stopped_due_to_obstacle": false
  },
  "error": null,
  "events": ["Moved forward 0.48 meters"],
  "timestamp": 1730001234.12,
  "safety": {
    "validated": true,
    "stopped_due_to_obstacle": false,
    "nearest_obstacle_m": 0.82
  }
}
```

---

### 5.13 `move_backward`

Bounded backward movement.

```python
move_backward(
    distance_m: float,
    speed_mps: float = 0.10
) -> ToolResult
```

#### Recommended safety limits

```text
max distance per command: 0.5 m
max speed: 0.15 m/s
requires rear/side safety if available
timeout required
```

---

### 5.14 `move_left`

Bounded lateral movement left.

```python
move_left(
    distance_m: float,
    speed_mps: float = 0.10
) -> ToolResult
```

---

### 5.15 `move_right`

Bounded lateral movement right.

```python
move_right(
    distance_m: float,
    speed_mps: float = 0.10
) -> ToolResult
```

---

### 5.16 `rotate`

Rotate in place.

```python
rotate(
    degrees: float,
    angular_speed_dps: float = 20.0
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "rotate",
  "result": {
    "requested_degrees": 30,
    "estimated_degrees": 29.2
  },
  "error": null,
  "events": ["Rotated 29.2 degrees"],
  "timestamp": 1730001234.12
}
```

---

### 5.17 `stand`

Put robot in standing pose.

```python
stand() -> ToolResult
```

---

### 5.18 `sit`

Put robot in sitting/resting pose.

```python
sit() -> ToolResult
```

---

### 5.19 `neutral_pose`

Put robot in neutral stance.

```python
neutral_pose() -> ToolResult
```

---

### 5.20 `low_inspection_pose`

Move body/camera into a better shelf-inspection posture.

```python
low_inspection_pose() -> ToolResult
```

#### Use case

Before:

```python
scan_shelf()
read_label(object_id)
```

---

### 5.21 `scan_pose`

Pose optimized for scene scanning.

```python
scan_pose() -> ToolResult
```

---

### 5.22 `demo_greeting_pose`

Optional expressive demo pose.

```python
demo_greeting_pose() -> ToolResult
```

---

## 6. Tier 2 — Strong demo / high-value autonomy tools

These make the project feel embodied and autonomous.

---

### 6.1 `track_person`

Track the user/person without moving.

```python
track_person() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "track_person",
  "result": {
    "person_visible": true,
    "distance_m": 1.4,
    "bearing_deg": -5.0
  },
  "error": null,
  "events": ["Person tracked"],
  "timestamp": 1730001234.12
}
```

---

### 6.2 `follow_person`

Follow the tracked user.

```python
follow_person(
    target_distance_m: float = 1.2,
    max_speed_mps: float = 0.2,
    timeout_s: float | None = None
) -> ToolResult
```

#### Behavior

- Uses person tracking.
- Maintains target distance.
- Stops if obstacle is too close.
- Writes follow status to memory.

---

### 6.3 `stop_following_person`

Stop the person-following behavior.

```python
stop_following_person() -> ToolResult
```

---

### 6.4 `reacquire_person`

Try to find the user again.

```python
reacquire_person(timeout_s: float = 5.0) -> ToolResult
```

#### Behavior

- Stop robot.
- Rotate or scan slowly.
- Look for person target.
- Report success/failure.

---

### 6.5 `go_to_location`

Navigate to a named place.

```python
go_to_location(name: str, timeout_s: float = 20.0) -> ToolResult
```

#### Example call

```json
{
  "name": "shelf"
}
```

#### Example return

```json
{
  "ok": true,
  "tool": "go_to_location",
  "result": {
    "location": "shelf",
    "nav2_status": "succeeded"
  },
  "error": null,
  "events": ["Arrived at shelf"],
  "timestamp": 1730001234.12
}
```

---

### 6.6 `save_current_location`

Save the current robot pose as a named place.

```python
save_current_location(name: str, description: str | None = None) -> ToolResult
```

#### Example call

```json
{
  "name": "shelf",
  "description": "Low product display table"
}
```

---

### 6.7 `list_known_locations`

List saved places.

```python
list_known_locations() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "list_known_locations",
  "result": {
    "locations": [
      {
        "name": "home",
        "description": "Starting point"
      },
      {
        "name": "shelf",
        "description": "Low product display table"
      }
    ]
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.12
}
```

---

### 6.8 `return_home`

Return to the saved home location.

```python
return_home() -> ToolResult
```

Equivalent to:

```python
go_to_location("home")
```

---

### 6.9 `turn_toward_object`

Turn robot or body toward a remembered object.

```python
turn_toward_object(object_id: str) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "turn_toward_object",
  "result": {
    "object_id": "obj_apple_1",
    "bearing_deg": -8.0,
    "turned_degrees": -8.0
  },
  "error": null,
  "events": ["Turned toward apple"],
  "timestamp": 1730001234.12
}
```

---

### 6.10 `center_object_in_view`

Visually align an object in the camera.

```python
center_object_in_view(
    object_id: str,
    timeout_s: float = 5.0
) -> ToolResult
```

Useful before label reading or approaching.

---

### 6.11 `approach_object`

Approach a remembered object.

```python
approach_object(
    object_id: str,
    stop_distance_m: float = 0.5,
    timeout_s: float = 8.0
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "approach_object",
  "result": {
    "object_id": "obj_apple_1",
    "final_distance_m": 0.51,
    "stopped_due_to_obstacle": false
  },
  "error": null,
  "events": ["Approached apple"],
  "timestamp": 1730001234.12,
  "safety": {
    "validated": true,
    "stopped_due_to_obstacle": false,
    "nearest_obstacle_m": 0.62
  }
}
```

---

### 6.12 `go_to_visible_object`

Find and approach an object from description.

```python
go_to_visible_object(
    description: str,
    stop_distance_m: float = 0.5
) -> ToolResult
```

#### Example call

```json
{
  "description": "the apple"
}
```

Internal chain:

```text
description → resolve object → estimate pose → approach_object
```

---

### 6.13 `get_visible_objects`

Return currently visible object memory.

```python
get_visible_objects() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "get_visible_objects",
  "result": {
    "objects": [
      {
        "object_id": "obj_apple_1",
        "label": "apple",
        "position": "center-left",
        "depth_m": 0.72,
        "bearing_deg": -8.0,
        "status": "visible"
      }
    ]
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.12
}
```

---

### 6.14 `get_object`

Return object memory for a specific object.

```python
get_object(object_id: str) -> ToolResult
```

---

### 6.15 `remember_object`

Write/update an object in memory.

```python
remember_object(update: dict) -> ToolResult
```

#### Example call

```json
{
  "update": {
    "object_id": "obj_granola_1",
    "attributes": {
      "contains_nuts": true
    },
    "risks": ["contains almonds"],
    "source": "vlm_label_read"
  }
}
```

Usually called by perception agent, not planner.

---

### 6.16 `find_objects_matching_constraints`

Semantic search over object memory.

```python
find_objects_matching_constraints(constraints: dict) -> ToolResult
```

#### Example call

```json
{
  "constraints": {
    "category": "snack",
    "avoid_allergens": ["nuts"],
    "prefer": ["healthy"]
  }
}
```

#### Example return

```json
{
  "ok": true,
  "tool": "find_objects_matching_constraints",
  "result": {
    "matches": [
      {
        "object_id": "obj_apple_1",
        "label": "apple",
        "score": 0.95,
        "reason": "Healthy whole food, no nut risk"
      }
    ]
  },
  "error": null,
  "events": ["Found apple matching constraints"],
  "timestamp": 1730001234.12
}
```

---

### 6.17 `set_active_goal`

Create or replace the active goal.

```python
set_active_goal(goal: dict) -> ToolResult
```

#### Example call

```json
{
  "goal": {
    "description": "Find a healthy snack without nuts",
    "constraints": {
      "avoid_allergens": ["nuts"],
      "prefer": ["healthy"]
    },
    "success_condition": "Recommend a visible safe snack"
  }
}
```

---

### 6.18 `get_active_goal`

Return the active goal.

```python
get_active_goal() -> ToolResult
```

---

### 6.19 `update_goal_status`

Update goal status.

```python
update_goal_status(
    goal_id: str,
    status: str,
    reason: str | None = None
) -> ToolResult
```

#### Status values

```text
pending
active
blocked
complete
failed
cancelled
```

---

## 7. Tier 3 — Advanced architecture / stretch tools

These add technical depth, stronger robotics credibility, and better assistive interaction.

---

### 7.1 `segment_scene`

Run segmentation.

```python
segment_scene(frame_id: str | None = None) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "segment_scene",
  "result": {
    "masks": [
      {
        "mask_id": "mask_001",
        "label": "apple",
        "confidence": 0.88,
        "bbox": [340, 210, 430, 320]
      }
    ]
  },
  "error": null,
  "events": ["Segmented scene"],
  "timestamp": 1730001234.12
}
```

---

### 7.2 `detect_text_regions`

Detect likely text areas.

```python
detect_text_regions(frame_id: str | None = None) -> ToolResult
```

---

### 7.3 `read_text_from_image`

OCR over a full frame or crop.

```python
read_text_from_image(
    frame_id: str | None = None,
    bbox: list[int] | None = None
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "read_text_from_image",
  "result": {
    "text": ["almond", "protein", "may contain nuts"],
    "confidence": 0.76
  },
  "error": null,
  "events": ["OCR completed"],
  "timestamp": 1730001234.12
}
```

---

### 7.4 `ask_vlm_about_scene`

Ask a VLM a question about the scene.

```python
ask_vlm_about_scene(
    question: str,
    frame_id: str | None = None
) -> ToolResult
```

#### Example call

```json
{
  "question": "Which visible items look like snacks?"
}
```

---

### 7.5 `ask_vlm_about_object`

Ask a VLM a question about a remembered object.

```python
ask_vlm_about_object(
    object_id: str,
    question: str
) -> ToolResult
```

---

### 7.6 `verify_object_identity`

Check whether an object is what memory says it is.

```python
verify_object_identity(
    object_id: str,
    expected_label: str
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "verify_object_identity",
  "result": {
    "object_id": "obj_apple_1",
    "expected_label": "apple",
    "verified": true,
    "confidence": 0.91
  },
  "error": null,
  "events": ["Verified obj_apple_1 as apple"],
  "timestamp": 1730001234.12
}
```

---

### 7.7 `verify_claim`

Check a semantic claim using available evidence.

```python
verify_claim(
    claim: str,
    object_id: str | None = None
) -> ToolResult
```

#### Example call

```json
{
  "claim": "The granola contains nuts",
  "object_id": "obj_granola_1"
}
```

---

### 7.8 `get_best_label_view`

Actively obtain the best frame/crop for label reading.

```python
get_best_label_view(
    object_id: str,
    attempts: int = 3
) -> ToolResult
```

#### Behavior

- Turn/center object.
- Capture multiple frames.
- Choose sharpest/front-facing crop.
- Return frame or crop ID.

#### Example return

```json
{
  "ok": true,
  "tool": "get_best_label_view",
  "result": {
    "object_id": "obj_granola_1",
    "crop_frame_id": "crop_887",
    "quality_score": 0.82
  },
  "error": null,
  "events": ["Captured best label view"],
  "timestamp": 1730001234.12
}
```

---

### 7.9 `capture_multi_view_scan`

Capture multiple views of a target area.

```python
capture_multi_view_scan(
    target: str = "shelf",
    angles: list[float] | None = None
) -> ToolResult
```

---

### 7.10 `look_at_object`

Turn body/camera toward an object to improve perception.

```python
look_at_object(object_id: str) -> ToolResult
```

---

### 7.11 `pixel_to_3d_ray`

Project image pixel into a 3D camera ray.

```python
pixel_to_3d_ray(
    x: int,
    y: int,
    camera: str = "front"
) -> ToolResult
```

---

### 7.12 `pixel_to_camera_point`

Project pixel plus depth into camera coordinates.

```python
pixel_to_camera_point(
    x: int,
    y: int,
    depth_m: float,
    camera: str = "front"
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "pixel_to_camera_point",
  "result": {
    "point_camera": {
      "x": 0.82,
      "y": -0.11,
      "z": 0.04
    }
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.12
}
```

---

### 7.13 `camera_point_to_base_link`

Transform a camera-frame point to `base_link`.

```python
camera_point_to_base_link(point: dict) -> ToolResult
```

---

### 7.14 `camera_point_to_map`

Transform a camera-frame point to `map`.

```python
camera_point_to_map(point: dict) -> ToolResult
```

---

### 7.15 `associate_pixel_with_lidar`

Relate a camera pixel to lidar geometry.

```python
associate_pixel_with_lidar(
    x: int,
    y: int,
    camera: str = "front"
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "associate_pixel_with_lidar",
  "result": {
    "pixel": [420, 260],
    "camera_depth_m": 0.82,
    "nearest_lidar_point": {
      "x": 0.78,
      "y": -0.12,
      "z": 0.31
    },
    "confidence": 0.74
  },
  "error": null,
  "events": ["Associated pixel with lidar point"],
  "timestamp": 1730001234.12
}
```

---

### 7.16 `estimate_object_pose`

Estimate object location relative to camera, robot, or map.

```python
estimate_object_pose(
    object_id: str,
    frame: str = "base_link"
) -> ToolResult
```

#### Supported frames

```text
camera
base_link
map
```

---

### 7.17 `refresh_object_position`

Update a remembered object's position using current perception.

```python
refresh_object_position(object_id: str) -> ToolResult
```

---

### 7.18 `reacquire_object`

Try to find a previously seen object.

```python
reacquire_object(
    object_id: str,
    timeout_s: float = 5.0
) -> ToolResult
```

#### Behavior

- Search recent expected area.
- Optionally rotate camera/body.
- Run detection/VLM.
- Update memory.

---

### 7.19 `track_object`

Continuously track a known object.

```python
track_object(
    object_id: str,
    timeout_s: float | None = None
) -> ToolResult
```

---

### 7.20 `detect_scene_change`

Detect whether the current scene differs from memory.

```python
detect_scene_change() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "detect_scene_change",
  "result": {
    "changed": true,
    "changes": [
      "Apple no longer visible",
      "New bottle visible on right"
    ]
  },
  "error": null,
  "events": ["Scene change detected"],
  "timestamp": 1730001234.12
}
```

---

### 7.21 `compare_current_scene_to_memory`

Compare current scene with semantic memory.

```python
compare_current_scene_to_memory() -> ToolResult
```

---

### 7.22 `describe_object_location`

Describe where an object is.

```python
describe_object_location(
    object_id: str,
    style: str = "plain"
) -> ToolResult
```

#### Styles

```text
plain
clockface
blind_assistive
robot_coordinates
```

#### Example return

```json
{
  "ok": true,
  "tool": "describe_object_location",
  "result": {
    "description": "The apple is slightly left of center, about 70 centimeters ahead.",
    "clock_direction": "10 o'clock",
    "distance_cm": 70
  },
  "error": null,
  "events": ["Described apple location"],
  "timestamp": 1730001234.12
}
```

---

### 7.23 `describe_location_for_blind_user`

Create an assistive location description.

```python
describe_location_for_blind_user(object_id: str) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "describe_location_for_blind_user",
  "result": {
    "spoken_instruction": "The apple is at your 10 o'clock, about 45 centimeters forward on the table.",
    "clock_direction": "10 o'clock",
    "distance_cm": 45,
    "height_description": "table height"
  },
  "error": null,
  "events": ["Generated blind-user location description"],
  "timestamp": 1730001234.12
}
```

---

### 7.24 `estimate_reachability`

Estimate whether the user can reach an object.

```python
estimate_reachability(
    object_id: str,
    user_position_known: bool = False
) -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "estimate_reachability",
  "result": {
    "reachable": true,
    "estimated_hand_distance_cm": 45,
    "risk": "low"
  },
  "error": null,
  "events": ["Estimated reachability"],
  "timestamp": 1730001234.12
}
```

---

### 7.25 `guide_user_hand_to_object`

Give step-by-step spoken hand guidance.

```python
guide_user_hand_to_object(object_id: str) -> ToolResult
```

#### Example spoken guidance

```text
Move your hand forward about 30 centimeters. Now slightly left. The apple should be under your fingertips.
```

---

### 7.26 `check_user_ready`

Check whether the user is ready for movement.

```python
check_user_ready() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "check_user_ready",
  "result": {
    "ready": true,
    "person_visible": true,
    "person_distance_m": 0.9,
    "reason": null
  },
  "error": null,
  "events": ["User appears ready"],
  "timestamp": 1730001234.12
}
```

---

### 7.27 `wait_until_user_ready`

Wait until the user is ready.

```python
wait_until_user_ready(timeout_s: float = 10.0) -> ToolResult
```

---

### 7.28 `is_user_nearby`

Check if user is nearby.

```python
is_user_nearby() -> ToolResult
```

---

### 7.29 `remember_place`

Create/update a semantic place memory.

```python
remember_place(
    name: str,
    description: str | None = None
) -> ToolResult
```

---

### 7.30 `describe_current_place`

Describe the current place.

```python
describe_current_place() -> ToolResult
```

---

### 7.31 `find_place_by_description`

Find a remembered place by description.

```python
find_place_by_description(query: str) -> ToolResult
```

#### Example call

```json
{
  "query": "the shelf with snacks"
}
```

---

### 7.32 `start_slam`

Start SLAM.

```python
start_slam() -> ToolResult
```

---

### 7.33 `stop_slam`

Stop SLAM.

```python
stop_slam() -> ToolResult
```

---

### 7.34 `save_map`

Save generated map.

```python
save_map(name: str) -> ToolResult
```

---

### 7.35 `load_map`

Load saved map.

```python
load_map(name: str) -> ToolResult
```

---

### 7.36 `get_map_status`

Return SLAM/map status.

```python
get_map_status() -> ToolResult
```

---

### 7.37 `start_nav2`

Start Nav2.

```python
start_nav2() -> ToolResult
```

---

### 7.38 `stop_nav2`

Stop Nav2.

```python
stop_nav2() -> ToolResult
```

---

### 7.39 `send_nav2_goal`

Send low-level Nav2 goal.

```python
send_nav2_goal(
    x: float,
    y: float,
    yaw: float,
    timeout_s: float = 20.0
) -> ToolResult
```

#### Access control

This should not normally be exposed to the main LLM planner. Prefer:

```python
go_to_location("shelf")
```

---

### 7.40 `cancel_nav2_goal`

Cancel active Nav2 goal.

```python
cancel_nav2_goal() -> ToolResult
```

---

### 7.41 `get_nav2_status`

Return Nav2 status.

```python
get_nav2_status() -> ToolResult
```

---

## 8. Tier 4 — Operator, debugging, fallback, and demo-rescue tools

These tools are not user-facing, but they are extremely useful in a hackathon demo.

---

### 8.1 `get_system_status`

Return overall system health.

```python
get_system_status() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "get_system_status",
  "result": {
    "backend": "ok",
    "rosbridge": "ok",
    "camera": "ok",
    "depth": "ok",
    "vlm": "ok",
    "tts": "ok",
    "nav2": "inactive"
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.12
}
```

---

### 8.2 `get_ros_topic_status`

Return ROS topic health.

```python
get_ros_topic_status() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "get_ros_topic_status",
  "result": {
    "/camera/rgb/image_raw": "active",
    "/camera/depth/image_raw": "active",
    "/cmd_vel": "active",
    "/detections/yolo": "stale"
  },
  "error": null,
  "events": [],
  "timestamp": 1730001234.12
}
```

---

### 8.3 `get_camera_status`

Return camera status.

```python
get_camera_status() -> ToolResult
```

---

### 8.4 `get_model_status`

Check local models and external model APIs.

```python
get_model_status() -> ToolResult
```

Should check:

```text
YOLO
segmentation
OCR
VLM
LLM
TTS
```

---

### 8.5 `get_last_error`

Return most recent system error.

```python
get_last_error() -> ToolResult
```

---

### 8.6 `run_self_check`

Run demo readiness check.

```python
run_self_check() -> ToolResult
```

#### Example return

```json
{
  "ok": true,
  "tool": "run_self_check",
  "result": {
    "checks": [
      {"name": "rosbridge", "status": "ok"},
      {"name": "rgb_camera", "status": "ok"},
      {"name": "depth_camera", "status": "ok"},
      {"name": "tts", "status": "ok"},
      {"name": "vlm", "status": "ok"}
    ],
    "ready_for_demo": true
  },
  "error": null,
  "events": ["Self-check passed"],
  "timestamp": 1730001234.12
}
```

---

### 8.7 `record_demo_clip`

Record a backup clip or sensor trace.

```python
record_demo_clip(
    name: str,
    duration_s: float = 30.0
) -> ToolResult
```

---

### 8.8 `save_successful_scan`

Save a known-good shelf scan for fallback.

```python
save_successful_scan(name: str) -> ToolResult
```

---

### 8.9 `replay_successful_scan`

Replay a known-good scan.

```python
replay_successful_scan(name: str) -> ToolResult
```

#### Use case

If live VLM/camera fails:

```text
Using my previous scan, I remember the apple was center-left.
```

---

### 8.10 `use_cached_scan`

Use the most recent or named cached scan.

```python
use_cached_scan(scan_id: str | None = None) -> ToolResult
```

---

### 8.11 `retry_last_action`

Retry the last failed action.

```python
retry_last_action() -> ToolResult
```

---

### 8.12 `fallback_to_stationary_mode`

Stop movement while keeping perception/dialogue alive.

```python
fallback_to_stationary_mode(reason: str) -> ToolResult
```

---

### 8.13 `fallback_to_text_mode`

Use text input/output if voice or TTS fails.

```python
fallback_to_text_mode(reason: str) -> ToolResult
```

---

### 8.14 `operator_override`

Operator sends a high-level command.

```python
operator_override(command: str, args: dict | None = None) -> ToolResult
```

#### Examples

```json
{
  "command": "stop"
}
```

```json
{
  "command": "use_cached_scan",
  "args": {
    "scan_id": "good_shelf_scan"
  }
}
```

---

### 8.15 `switch_to_fake_robot`

Switch backend to mock robot.

```python
switch_to_fake_robot() -> ToolResult
```

---

### 8.16 `switch_to_real_robot`

Switch backend to real robot.

```python
switch_to_real_robot() -> ToolResult
```

---

### 8.17 `pause_agent`

Pause autonomous agent loop.

```python
pause_agent() -> ToolResult
```

---

### 8.18 `resume_agent`

Resume autonomous agent loop.

```python
resume_agent() -> ToolResult
```

---

### 8.19 `mute_robot`

Mute robot speech/audio.

```python
mute_robot() -> ToolResult
```

---

### 8.20 `unmute_robot`

Unmute robot speech/audio.

```python
unmute_robot() -> ToolResult
```

---

## 9. Tier summary

### 9.1 Tier 0 — must exist

```python
stop()
emergency_stop()
check_safety()
get_robot_status()
cancel_current_action()
reset_safety_latch()
```

---

### 9.2 Tier 1 — MVP demo-critical

```python
speak()
display_plan()

get_rgb_frame()
get_depth_frame()
get_depth_at_pixel()

detect_objects()
describe_scene()
scan_shelf()
read_label()

find_object()
resolve_reference()

move_forward()
move_backward()
move_left()
move_right()
rotate()

stand()
sit()
neutral_pose()
low_inspection_pose()
scan_pose()
demo_greeting_pose()
```

---

### 9.3 Tier 2 — strong demo

```python
track_person()
follow_person()
stop_following_person()
reacquire_person()

go_to_location()
save_current_location()
list_known_locations()
return_home()

turn_toward_object()
center_object_in_view()
approach_object()
go_to_visible_object()

get_visible_objects()
get_object()
remember_object()
find_objects_matching_constraints()

set_active_goal()
get_active_goal()
update_goal_status()
```

---

### 9.4 Tier 3 — advanced / stretch

```python
segment_scene()
detect_text_regions()
read_text_from_image()

ask_vlm_about_scene()
ask_vlm_about_object()
verify_object_identity()
verify_claim()

get_best_label_view()
capture_multi_view_scan()
look_at_object()

pixel_to_3d_ray()
pixel_to_camera_point()
camera_point_to_base_link()
camera_point_to_map()
associate_pixel_with_lidar()

estimate_object_pose()
refresh_object_position()
reacquire_object()
track_object()

detect_scene_change()
compare_current_scene_to_memory()

describe_object_location()
describe_location_for_blind_user()
estimate_reachability()
guide_user_hand_to_object()

check_user_ready()
wait_until_user_ready()
is_user_nearby()

remember_place()
describe_current_place()
find_place_by_description()

start_slam()
stop_slam()
save_map()
load_map()
get_map_status()

start_nav2()
stop_nav2()
send_nav2_goal()
cancel_nav2_goal()
get_nav2_status()
```

---

### 9.5 Tier 4 — demo rescue / operator tools

```python
get_system_status()
get_ros_topic_status()
get_camera_status()
get_model_status()
get_last_error()
run_self_check()

record_demo_clip()
save_successful_scan()
replay_successful_scan()
use_cached_scan()

retry_last_action()
fallback_to_stationary_mode()
fallback_to_text_mode()

operator_override()
switch_to_fake_robot()
switch_to_real_robot()
pause_agent()
resume_agent()
mute_robot()
unmute_robot()
```

---

## 10. LLM-facing tool subsets

Do not expose all tools directly to all agents.

---

### 10.1 Main planner tools

The main planner should see a curated set.

```python
PLANNER_TOOLS = [
    "get_robot_status",
    "check_safety",
    "speak",
    "display_plan",

    "get_visible_objects",
    "find_object",
    "resolve_reference",
    "find_objects_matching_constraints",

    "describe_scene",
    "scan_shelf",
    "read_label",

    "set_active_goal",
    "get_active_goal",
    "update_goal_status",

    "go_to_location",
    "follow_person",
    "turn_toward_object",
    "approach_object",

    "stop"
]
```

---

### 10.2 Perception agent tools

```python
PERCEPTION_TOOLS = [
    "get_rgb_frame",
    "get_depth_frame",
    "get_depth_at_pixel",

    "detect_objects",
    "segment_scene",
    "detect_text_regions",
    "read_text_from_image",

    "ask_vlm_about_scene",
    "ask_vlm_about_object",
    "verify_object_identity",

    "remember_object",
    "refresh_object_position"
]
```

---

### 10.3 Safety supervisor tools

```python
SAFETY_TOOLS = [
    "check_safety",
    "get_robot_status",
    "stop",
    "emergency_stop",
    "cancel_current_action",
    "cancel_nav2_goal"
]
```

---

### 10.4 Operator tools

```python
OPERATOR_TOOLS = [
    "run_self_check",
    "get_system_status",
    "get_ros_topic_status",
    "save_successful_scan",
    "replay_successful_scan",
    "fallback_to_stationary_mode",
    "operator_override",
    "pause_agent",
    "resume_agent"
]
```

---

## 11. Recommended implementation order

Implement in this order:

```text
1. stop, emergency_stop, check_safety, get_robot_status
2. speak, display_plan
3. get_rgb_frame, get_depth_frame, get_depth_at_pixel
4. detect_objects
5. scan_shelf
6. read_label
7. object memory: get_visible_objects, find_object, resolve_reference
8. move_forward, rotate, low_inspection_pose
9. follow_person or go_to_location
10. approach_object / turn_toward_object
11. fallback tools: save_successful_scan, use_cached_scan
12. advanced grounding: pixel_to_3d, lidar association, reachability
```

---

## 12. Absolute indispensable set

If time is very tight, build only this set:

```python
stop()
check_safety()
get_robot_status()

speak()
display_plan()

get_rgb_frame()
get_depth_at_pixel()

detect_objects()
scan_shelf()
read_label()

get_visible_objects()
find_object()
resolve_reference()

move_forward()
rotate()
turn_toward_object()
```

This supports the core demo:

```text
User: What is on the shelf?
Robot: I see an apple, granola, and orange juice.

User: Anything with nuts?
Robot: The granola appears to contain almonds.

User: Which is healthiest?
Robot: The apple is the safest healthy option.

User: Take me to it.
Robot: The apple is slightly left. Turning toward it now.
```

---

## 13. Suggested code organization

```text
backend/
  agents/
    dialogue_agent.py
    planner_agent.py
    perception_agent.py
    safety_supervisor.py

  memory/
    store.py
    schemas.py
    object_registry.py
    goal_manager.py
    event_log.py

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

  robot/
    rosbridge_client.py
    nav2_client.py
    camera_client.py
    depth_client.py
    cmd_vel_client.py
    person_tracking_client.py
    fake_robot.py

  ui/
    websocket_server.py
    event_stream.py
```

---

## 14. Recommended tool registry interface

```python
class Tool:
    name: str
    tier: int
    description: str
    allowed_callers: list[str]
    requires_safety_check: bool

    async def run(self, args: dict, context: "ToolContext") -> ToolResult:
        raise NotImplementedError
```

```python
@dataclass
class ToolContext:
    memory: MemoryStore
    robot: RobotClient
    safety: SafetySupervisor
    event_stream: EventStream
    frame_buffer: FrameBuffer
```

---

## 15. Access-control recommendation

| Tool class | Main planner | Perception agent | Safety supervisor | Operator |
|---|---:|---:|---:|---:|
| Speak/display | Yes | No | Yes, for safety speech | Yes |
| RGB/depth | Limited | Yes | No | Yes |
| VLM/OCR | Limited | Yes | No | Yes |
| Movement | Limited | No | Validate only | Yes |
| Emergency stop | Yes | No | Yes | Yes |
| Reset safety | No | No | No | Yes |
| Raw Nav2 goal | No by default | No | No | Yes |
| Cached scan/replay | Yes | Yes | No | Yes |
| Fake/real robot switch | No | No | No | Yes |

---

## 16. Demo-critical sequence supported by the tools

### User request

```text
Find me a healthy snack without nuts.
```

### Tool flow

```text
set_active_goal()
display_plan()
check_safety()
go_to_location("shelf")             # optional if Nav2 works
low_inspection_pose()
scan_shelf()
find_objects_matching_constraints()
read_label()                        # if needed
speak("The apple is the safest healthy option.")
resolve_reference("it")             # for follow-up
turn_toward_object("obj_apple_1")
describe_location_for_blind_user()  # if implemented
```

### Follow-up

```text
User: Take me to it.
```

Tool flow:

```text
resolve_reference("it")
check_safety()
turn_toward_object(object_id)
approach_object(object_id)          # if safe and implemented
speak("It is in front of you, slightly to the left.")
```

---

## 17. Notes on safe planner behavior

The planner should prefer:

```python
go_to_location("shelf")
approach_object("obj_apple_1")
turn_toward_object("obj_apple_1")
```

over:

```python
send_nav2_goal(x, y, yaw)
move_forward(0.73)
rotate(-13.2)
```

Low-level tools can exist, but high-level semantic skills are safer and easier to validate.

---

## 18. Notes on VLM use

Good uses:

```text
describe_scene()
scan_shelf()
read_label()
ask_vlm_about_object()
verify_claim()
```

Bad uses:

```text
high-frequency steering
obstacle avoidance
motor control
continuous person tracking
real-time safety decisions
```

The VLM should enrich memory. It should not be in the motor loop.

---

## 19. Notes on memory

Object memory should enable:

```text
User: What did you see?
User: Which one has nuts?
User: Which is healthiest?
User: Take me to it.
User: Where is the apple?
```

Recommended object memory fields:

```python
ObjectMemory = {
    "id": str,
    "label": str,
    "confidence": float,
    "last_seen_ts": float,
    "seen_count": int,

    "bbox": tuple[int, int, int, int] | None,
    "mask_id": str | None,
    "depth_m": float | None,
    "bearing_deg": float | None,

    "position_text": str | None,
    "place": str | None,

    "visible_text": list[str],
    "attributes": dict,
    "risks": list[str],

    "sources": list[str],
    "status": "visible | occluded | stale | removed"
}
```

---

## 20. Final implementation recommendation

For the hackathon, the highest-value tool path is:

```text
scan_shelf()
→ object memory
→ read_label()
→ find_objects_matching_constraints()
→ speak()
→ resolve_reference()
→ turn_toward_object()
```

This path directly demonstrates the novel value proposition:

```text
The robot does semantic last-meter assistance:
it sees objects, remembers them, reads labels, reasons over constraints,
answers follow-up questions, and guides the user toward the chosen item.
```
