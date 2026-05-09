# Assistive Guide Dog Hackathon — Planning Document

A robotics hackathon project: a quadruped robot acting as an AI-augmented assistant for blind / low-vision users, focused on the "last meter" gap that real guide dogs and white canes don't address — identifying specific products, reading labels, and answering questions about the immediate environment.

---

## 1. Context

- **Duration**: ~1.5 days (~12–16 productive hours after setup, debugging, sleep, and the demo itself).
- **Team**: 3 MSc students.
- **Robot**: DeepRobotics Jueying Lite3 with multiple RGB cameras, RGBD camera, and a Livox Mid-360 3D lidar mounted on top.
- **Compute**: Onboard + laptop on same network. Cloud API access available for LLM/VLM.
- **Robot SDK experience**: None on this specific platform (some prior Unitree experience transfers conceptually but not directly).

---

## 2. Problem framing

A real guide dog handles the *local* navigation problem (don't hit things, stop at curbs, follow a learned route). The handler is the navigator who decides where to go. The dog explicitly does not read signs, identify products, or interpret semantic information.

This leaves a documented gap — the **"last meter" problem**: blind users can get to the shop, but identifying which can is soup vs. tomato sauce, reading the expiration date, comparing two products, or finding a specific item on a shelf still requires a sighted human. Recent HRI research with BVI participants confirms this is where current robotic guide systems fall short.

The project's value proposition is therefore *not* "we built a guide dog" — that invites comparison with real guide dogs and we lose. It is "we built a guide dog assistant that does what guide dogs cannot: semantic perception, label reading, multi-turn reasoning about the environment."

The framing also adjusts who navigates: the **user is the navigator**, the dog is the executor + perceiver. The user provides high-level intent ("let's go to the shelf"); the dog handles execution and answers questions about what it sees.

---

## 3. What the demo needs to do

### Hackathon judging realities

Judges evaluate roughly:

1. **Demo presence** — is it impressive in the moment, does it tell a story, is there a wow.
2. **Vision and architecture** — coherent system design, thoughtful choices, plausible path to something real.
3. **Technical depth** — real engineering, not just glued APIs.
4. **Storytelling** — why does this matter, who is it for, what did the team figure out.

What is *not* on the list: every advertised feature working flawlessly. Judges discount hackathon code; they reward calibrated teams who say "here's what works, here's what's mocked, here's why."

### Implication

The architecture you describe ≠ the architecture you implement. Describe the full vision; demo a polished subset; be explicit about what's mocked. This is honest, not evasive, and it scores better than overpromising.

---

## 4. Demo design

**Slot**: ~3:30 of demo + pitch.
**Setting**: Indoor, with a low table (~50cm) acting as a "shop shelf." Items chosen for clear, large, readable labels. Dog's eye height (~knee level) means a normal grocery shelf doesn't work — the low fixture is a deliberate choice and is disclosed in the pitch.
**Props**: Low table, 5–6 grocery-style items with prominent labels, harness pole (PVC + foam grip), blindfold, laptop on stand, phone running the web app.

### Roles during demo

- **U** — user, blindfolded, holds harness pole.
- **N** — narrator/operator, runs laptop, watches for trouble, holds e-stop, delivers framing and pitch.
- **D** — the dog's voice (TTS through laptop or phone).

### Beat-by-beat script

**Beat 1 — Framing (0:00–0:25)**
N to judges: "Two percent of blind people who could benefit from a guide dog actually have one — the wait list is years long. And even those who have one still need a sighted human for one specific thing: identifying products at point of purchase. That's the gap we're closing. Watch."

**Beat 2 — Intent (0:25–0:40)**
U: "Hey, I want to find a healthy snack at the shop. Can you take me?"
Phone shows: *Listening → Thinking → Plan: navigate to shelf, then assist with selection.*
D: "Sure. The shelf is about three meters ahead. Let's go."

**Beat 3 — Transit with live narration (0:40–1:10)**
Dog walks toward shelf under agent control (velocity primitives composed by the agent). VLM narration during the walk:
D: "Path is clear ahead… approaching the display now… we're here."
The narration uses real VLM calls; the locomotion is real agent-controlled motion (Level 2; see §5).

**Beat 4 — Shelf scan (1:10–1:35)**
U: "What's on the shelf?"
Phone: *Plan: scan_shelf().*
Dog rotates body slowly across the table.
D: "I see five items. From left to right: a box of granola, a can of tomato soup, an apple, a chocolate bar, and a bottle of orange juice."

**Beat 5 — Multi-turn grounded reasoning (1:35–2:30)** — *centerpiece*
U: "Anything with nuts? I'm allergic."
D: "Checking labels… the granola has almonds. The chocolate bar lists 'may contain nuts.' The others are nut-free."

U: "Which is the healthiest non-nut option?"
D: "Of the nut-free items: the apple is whole fruit with no processing. The soup is low in calories but high in sodium. I'd suggest the apple."

U: "Take me to it."
D: "The apple is to your left. Walking now."
Dog turns and walks toward the apple (visual servo or short scripted move).
D: "It's right in front of you, on the table."

**Beat 6 — Return (2:30–2:55)**
U: "Thanks, let's head back."
D: "Heading home."
Dog walks back. Brief narration. Arrives.

**Beat 7 — Pitch (2:55–3:30)**
N: "Under the hood: voice in via Whisper, an LLM agent with eight tools, vision-language model on a frame stream from the dog's camera, and a Livox Mid-360 lidar feeding a perception stack. The locomotion is agent-controlled velocity primitives composed by the LLM. The architecture also supports Nav2-based autonomous navigation with the lidar — here's a clip [show video]. Happy to dive into any layer."

### Tier 1 / Tier 2 split

Because the SDK is unfamiliar, the demo is structured so that locomotion is the *last* thing added, not the first:

- **Tier 1 (must work, ~2 min)**: Beats 1, 2, 4, 5, 7. Stationary dog, perception-only. Frame as "user has arrived at the shelf with their assistant." Shorter but complete vignette.
- **Tier 2 (adds ~1 min)**: Beats 3 and 6 (transit). Requires SDK locomotion working.

Tier 1 is built and rehearsed by Day 1 evening. Tier 2 is added Day 2 morning if locomotion is up. The transition is purely additive — no redesign needed if locomotion fails.

---

## 5. Locomotion strategy: Level 2 (agent-controlled primitives)

Levels considered:

1. **Level 1**: Agent triggers pre-recorded velocity sequences. Reliable but uninteresting for a robotics hackathon.
2. **Level 2** *(target)*: Agent issues parameterized motion commands (`walk_forward(m)`, `turn(deg)`, `walk_to_visual_target(desc)`). Built on top of `set_velocity(vx, vy, omega)` from the SDK. Composes well with the agent's tool-calling.
3. **Level 3** *(stretch)*: Nav2 with Livox lidar (FAST-LIO for odometry, pre-built occupancy grid). Real autonomous navigation. Estimated 40–50% probability of coming together in time.
4. **Level 4**: VLM-as-controller, end-to-end. Out of scope.

**Primary commitment: Level 2.** Stretch toward Level 3 in parallel. Fall back to Level 1 only if SDK fails the Day 1 morning gate.

### Level 2 primitives

All built from `set_velocity()`:

- `walk_forward(meters)` — set forward velocity, sleep, stop. Dead reckoning, ±10cm accuracy is fine.
- `turn(degrees)` — angular velocity, timed.
- `walk_to_visual_target(description)` — loop at 2–5Hz: get camera frame → VLM returns bearing ("left/right/centered/arrived") → set angular velocity proportional to bearing.
- `stop()` — zero velocity.

### Day 1 morning hard gate

By **12:30 Day 1**, the team must be able to send a velocity command from Python and have the dog physically move. If yes → Level 2 is the plan. If no → escalate to organizers, parallel-track Level 1, reassess at 14:00. Do **not** keep grinding on the SDK past lunch — that's how the project gets lost.

### Don't pursue Level 3 if Level 2 isn't working

Nav2 is strictly more complex than velocity primitives. If `set_velocity` doesn't work, Nav2 has zero chance.

---

## 6. Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Phone (web app)                            │
│  Push-to-talk · live transcript · agent state · plan display    │
└──────────────────────────────┬──────────────────────────────────┘
                               │ websocket
┌──────────────────────────────▼──────────────────────────────────┐
│                          Laptop                                 │
│                                                                 │
│   Whisper (STT) ──────────┐                                     │
│                           ▼                                     │
│              ┌────────────────────────┐                         │
│              │  LLM Agent (Claude)    │                         │
│              │  · system prompt       │                         │
│              │  · ~8 tools            │                         │
│              │  · structured plans    │                         │
│              │  · conversation memory │                         │
│              └────────┬───────────────┘                         │
│                       │                                         │
│      ┌────────────────┼─────────────────┬──────────────┐        │
│      ▼                ▼                 ▼              ▼        │
│   VLM tools     Locomotion tools    speak()       get_status()  │
│   (Claude/GPT   (velocity prims,    (TTS)                       │
│   vision API)   visual servo)                                   │
│      │                │                                         │
│      │                ▼                                         │
│      │         RobotInterface (FakeRobot │ Lite3SDK)            │
│      │                │                                         │
└──────┼────────────────┼─────────────────────────────────────────┘
       │                │
       │ camera frames  │ velocity commands
       ▼                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Jueying Lite3                                │
│   RGB cameras · RGBD · Livox Mid-360 lidar · motion SDK         │
└─────────────────────────────────────────────────────────────────┘

  Designed but not on the demo critical path:
  · Nav2 + FAST-LIO on Livox lidar (Level 3 stretch goal)
  · Hardware e-stop
  · Outdoor route learning (visual teach-and-repeat)
```

### Agent design

**One agent loop, ReAct-style, with a small tool surface.** Modern LLMs (2026 frontier) do tool selection and multi-step planning reliably; we don't need separate planner/executor stages, behavior trees, or PDDL. The LLM is the planner. Tool design is where 80% of the work and 80% of the value lives.

**Structured plan emission**: before executing, the agent emits a plan object (intent + ordered steps + tool calls). This is rendered live on the phone UI — judges see the reasoning happen. The plan structure is also useful for debugging and for graceful failure (a step can fail and the agent replans).

**Direct command bypass**: short-circuit voice commands like "stop" with a regex check before they hit the LLM. Keeps latency-critical safety responses sub-second.

### Tool surface (target: ~8 tools)

| Tool                              | Purpose                                                     | Critical path |
|-----------------------------------|-------------------------------------------------------------|---------------|
| `walk_forward(meters)`            | Move forward a distance                                     | yes           |
| `turn(degrees)`                   | Rotate in place                                             | yes           |
| `walk_to_visual_target(desc)`     | Visual-servo toward a described target                      | important     |
| `stop()`                          | Zero velocity, halt                                         | yes (safety)  |
| `describe_scene()`                | VLM call on current frame, free-form description            | yes           |
| `scan_shelf()`                    | Slow body rotation + VLM, returns structured item list      | yes           |
| `read_label(item_ref)`            | VLM focused on text on a specific item from the scan        | yes           |
| `speak(text)`                     | TTS output                                                  | yes           |
| `get_status()`                    | Battery, current task, fallback queries                     | nice-to-have  |

`scan_shelf` outputs structured data: `[{name, position: "leftmost"|"second from left"|..., bearing_deg, description}]`. Subsequent tools (`read_label`, `walk_to_visual_target`) reference items from this output. This is the spatial-memory mechanism — the scan result lives in the agent's context, follow-up questions are answered from it without re-scanning.

### Key architectural principles

- **The LLM should not be in the control loop.** It decides intent and calls tools. If the LLM hangs or hallucinates, the dog stands still — it doesn't walk into a wall.
- **`RobotInterface` abstraction.** All robot interaction goes through one class with a `FakeRobot` and a `Lite3SDK` implementation. Team develops against the mock; swap to real on Day 1 afternoon. Critical for parallelization with 3 people.
- **Latency is part of the UX.** VLM calls are 2–5s. Agent says "let me take a look…" before the call returns. Pre-fetch where the demo script is predictable (e.g., kick off `scan_shelf` as soon as the dog arrives at the table).
- **Plan visibility on the phone is a feature, not a debug tool.** Judges should be able to read what the agent is about to do, in plain English, before it does it.

---

## 7. Roles (3 people)

Specialization, not pair-programming. Each person owns a vertical with a clean interface to the others.

**Person A — Robot / locomotion**
Owns: SDK bringup, `Lite3SDK` implementation of `RobotInterface`, camera streaming into a Python frame buffer, the locomotion tools (`walk_forward`, `turn`, `walk_to_visual_target`, `stop`), Day 2 morning route recording / fixture setup. If A is the team member with prior Unitree experience, that's the right fit.

**Person B — Agent / perception**
Owns: LLM agent loop, system prompt, tool registration, VLM-based perception tools (`describe_scene`, `scan_shelf`, `read_label`), prompt engineering for spatial memory and multi-turn reasoning, structured plan emission. **This is the intellectual core of the demo** — Beats 4 and 5 stand or fall on this work.

**Person C — Voice / UI / pitch**
Owns: phone web app (push-to-talk, live transcript, plan display, agent state indicator, optional camera view), Whisper integration, TTS, network setup, **architecture diagram and pitch script from Day 1**. Runs the laptop during the demo. Person C is the integration glue and the demo MC.

---

## 8. Day-by-day plan

### Day 0 — Pre-hackathon prep (highest leverage)

The single biggest risk-reducer. Skipping this is how the project loses Day 1.

1. **Find someone who has used a Lite3.** Lab mates, organizers, distributor. A 30-minute call beats 5 hours of docs. Specific questions: how do I send a velocity command from Python? How do I read a camera frame? What state must the robot be in? What's the network config?
2. **Locate the SDK on GitHub before the hackathon.** Search `lite3_motion_sdk`, `deeprobotics`, `Jueying`. Read the README. Identify the exact velocity command function. Have it written down on paper before Day 1 morning.
3. **30 minutes with the actual robot if at all possible.** Confirm "I can connect and send one command." This single confirmation collapses the biggest single risk.
4. **Build the `FakeRobot` mock with the API the team expects to need.** Test the agent loop against it the night before.
5. **Smoke-test one VLM call** with a structured-output prompt on a real photo of a cluttered table. Confirm clean JSON back. Removes the biggest non-robot risk.

### Day 1 morning (3–4 hours)

- **Person A**: SDK bringup. **Hard gate at 12:30**: dog moves from a Python script. If yes → Level 2 plan continues. If no → escalate, fall back to Level 1, reassess at 14:00.
- **Person B**: agent loop + tool registration against `FakeRobot`. Voice → agent → fake tools → TTS working end-to-end. Should be able to "narrate a fake walk" by lunch.
- **Person C**: phone web app skeleton, push-to-talk, transcript display. Mock agent state stream from canned data initially.

### Day 1 afternoon (4–5 hours)

- **A**: swap `FakeRobot` for real Lite3 implementation. First end-to-end test ("user says go to shelf, dog walks under agent control with `walk_forward`, narrates with real VLM") by ~17:00. **This is the Day 1 success gate.**
- **B**: starts on `scan_shelf` and `read_label`. Pure VLM prompt-engineering work, can be done with photos before integration.
- **C**: plan display on phone, agent state indicator, integration with B's structured plan output.

### Day 1 evening (3–4 hours)

- **B**: spatial memory polish, multi-turn reasoning, demo Beat 5 working end-to-end. **The centerpiece — get it polished while everyone's awake, not at 3am.**
- **A**: `walk_to_visual_target` visual servo, optional Nav2 / FAST-LIO exploration in parallel (Level 3 stretch — only if Level 2 is solid).
- **C**: architecture diagram first draft, pitch script first draft.

### Day 2 morning (3–4 hours)

- All: set up actual demo space — low table, items, harness pole, marked floor positions.
- A: record any necessary scripted segments (transit fallback, tier 1 framing).
- A: 90-minute lidar pitch task — set up Livox SDK, RViz visualization, record 30s of pointcloud streaming during a manual walk for the architecture pitch (independent of motion SDK status, well-supported by Livox).
- All: **run the full demo script start to finish.** Time it. Note what breaks.
- All: fix what broke. Add fallbacks (VLM timeout → "let me look again"; TTS fail → text-only on phone).

### Day 2 afternoon — pre-demo

- **Code freeze ~2 hours before demo time.** After freeze: bug fixes only, no features.
- Run the demo end-to-end 5+ times.
- One team member plays "judge asking weird questions" to test edge cases.
- Charge the dog. Test the network. Have a backup laptop or hotspot.
- **Record a backup video of a successful run *now***, in case live demo fails.

---

## 9. Risk register

| Risk                                       | Probability | Mitigation                                                                                                          |
|--------------------------------------------|-------------|---------------------------------------------------------------------------------------------------------------------|
| SDK setup eats Day 1                       | medium-high | Day 0 prep, hard 12:30 gate, Tier 1 demo doesn't need locomotion                                                    |
| Camera streaming flaky                     | medium      | Phone strapped to dog as backup (modern phone camera + RTSP/WebRTC, ~1 hour to set up)                              |
| VLM latency too high                       | medium      | Pre-warm, cache scan, agent narrates during waits                                                                   |
| Voice loop unreliable                      | low-medium  | Fallback to text input on phone                                                                                     |
| Dog falls or stalls during demo            | low-medium  | E-stop, restart, slow down user pace if needed                                                                      |
| Network drops mid-demo                     | low         | Phone hotspot, two laptops                                                                                          |
| Whole live demo fails                      | low         | Backup video recorded Day 2 morning; pitch the architecture and play the video                                      |
| Visual servo (`walk_to_visual_target`) flaky | medium    | Fall back to scripted short move (`turn(N); walk_forward(M)`) using bearing from scan                               |
| Nav2 / FAST-LIO doesn't come together      | medium-high | It's a stretch goal, not a commitment. Lidar still appears in the pitch as a 30-second pointcloud video.            |

---

## 10. Three commitments before coding starts

1. **Centerpiece is Beat 5 (multi-turn shelf reasoning).** If something has to get cut, it's not Beat 5.
2. **Locomotion is Level 2 (agent-controlled velocity primitives), with Level 3 as parallel stretch and Level 1 as fallback only if the Day 1 morning gate fails.**
3. **Person C owns the architecture diagram and pitch from Day 1.** It is a deliverable, not an afterthought.

---

## 11. Pitch hygiene (honesty as a feature)

Things that score well with knowledgeable judges:

- "Here's what we don't claim to solve: outdoor autonomous navigation, certified safety, real-time obstacle avoidance at running pace."
- "Here's what we claim: an architecture for AI-augmented assistive guidance, with a working subset demonstrating the novel perception + reasoning layer."
- "Outdoor nav is the right architectural choice but a 6-month engineering problem. The agent interface treats it identically to the velocity primitives we shipped, so swapping is a one-day integration."
- "We chose the Lite3 because [reason]; the architecture is platform-agnostic, with the SDK abstracted behind a clean interface."

If a judge asks "is this autonomous or scripted?", the answer should be specific and unembarrassed: "the velocity primitives are real and agent-composed; the routes the agent walks are short and bounded; full autonomous nav with our lidar is the architectural target, prototyped but not on the demo critical path." Calibration > hype.
