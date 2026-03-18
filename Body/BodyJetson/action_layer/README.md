
# Action Layer

The **Action Layer** is responsible for executing robot behaviors (skills) in a structured and controllable way.

It sits between:
- **Perception** (e.g. tracking, detection)
- **Low-level control** (motor commands)
- **Future planner** (high-level decisions)

---

## Concept

The Action Layer provides:
- A **central behavior manager node**
- A **plugin-like skill system**
- A **resource-based scheduling system**
- A **ROS2 action interface** for external control

It ensures that:
- Only compatible behaviors run at the same time
- Behaviors can be started, stopped, and monitored
- Motor control remains predictable and safe

---

## Architecture Overview

```

Planner / UI / Controller
│
▼
ROS2 Actions (e.g. follow_track)
│
▼
behavior_manager_node  ← ROS interface
│
▼
BehaviorManager      ← scheduling + lifecycle
│
▼
Skills           ← actual logic (e.g. follow_track)
│
▼
(Motor) Commands (/motor_command)

````

---

## Core Principles

### 1. Skills are modular
Each behavior is implemented as a standalone **skill**.

### 2. Manager controls execution
The `BehaviorManager`:
- Starts/stops skills
- Prevents conflicts via **resource tags**
- Runs the update loop (10 Hz)

### 3. Node handles ROS
The `behavior_manager_node`:
- Handles actions, topics, timers
- Publishes outputs
- Feeds inputs into the manager

### 4. Resource-based safety
Skills declare required resources (e.g. `"head_motion"`).  
The manager prevents conflicting behaviors from running simultaneously.

---

## Execution Model

- **10 Hz loop** → updates active skills
- **~2 Hz loop** → publishes system status
- **Action interface** → start/stop behaviors

---

## Current Features

- Follow a tracked object via `/follow_track` action
- Head motor control using velocity commands
- Resource conflict prevention (basic)

---

## Future Extensions

- Multiple simultaneous skills
- Priority system / arbitration
- Planner integration
- More sensor inputs (face, posture, etc.)

---

## Running

```bash
docker compose up -d action_layer
docker logs -f action_layer
````

Send a goal:

```bash
ros2 action send_goal /follow_track behavior_msgs/action/FollowTrack "{track_id: 15}" --feedback
```

---

## Notes

* This layer is **execution-only** (no planning logic)
* It is designed to grow with increasing system complexity
* Skills should be simple and composable

