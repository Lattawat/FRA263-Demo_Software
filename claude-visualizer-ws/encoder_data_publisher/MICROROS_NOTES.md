# micro-ROS Notes (Teensy 4.1 / Arduino / PlatformIO)

---

## 1. micro-ROS Installation

### System Prerequisites
Before building, the following must be in place:
- Ubuntu 22.04/24.04 (or WSL2)
- ROS2 Jazzy installed and sourced: `source /opt/ros/jazzy/setup.bash`
- Python build tools:
  ```bash
  sudo apt install python3-colcon-common-extensions python3-pip cmake
  ```
- PlatformIO installed (VS Code extension or `pip install platformio`)

**Why:** micro_ros_platformio's build script calls `colcon build` internally to
cross-compile the micro-ROS C library for Teensy's ARM Cortex-M7. Without a
sourced ROS2 environment and colcon, the build fails even before touching your code.

### Configure platformio.ini
```ini
lib_deps = https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = jazzy        ; must match your host ROS2 distro
board_microros_transport = serial    ; USB serial transport
```

- `lib_deps`: PlatformIO downloads and manages the library from GitHub
- `board_microros_distro`: selects which version of message definitions are generated —
  must match the ROS2 distro on the host PC running the agent
- `board_microros_transport`: wires the correct serial/UDP/etc. transport layer

### First Build
```bash
~/.platformio/penv/bin/pio run
```
The first build takes 5–15 minutes. It:
1. Downloads micro_ros_platformio
2. Runs `colcon build` using the Teensy ARM cross-compiler
3. Generates all standard message headers (std_msgs, geometry_msgs, etc.)
4. Produces a static `.a` library that is linked into your firmware

**Why use `~/.platformio/penv/bin/pio`:** The VS Code PlatformIO extension installs
its own isolated Python environment. Using the system `pio` (different version/path)
can produce inconsistent builds.

### Install micro-ROS Agent (Host PC)
The agent bridges micro-ROS on Teensy to the full ROS2 network on the PC.

**Option A — Docker (simplest):**
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200
```

**Option B — Build from source in your ROS2 workspace:**
```bash
# source your workspace, then:
colcon build --packages-select micro_ros_agent
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

---

## 2. micro-ROS Fundamentals (Must Know)

### ROS2 Layer Overview
```
Your Application Code
         ↓
rclcpp / rclpy / rclc     ← language bindings
         ↓
rcl  (ROS Client Library — C)   ← nodes, topics, services, timers
         ↓
rmw  (middleware interface)     ← vendor-neutral DDS abstraction
         ↓
DDS  (FastDDS / CycloneDDS)     ← actual network transport
```
In micro-ROS, `rclc` is used instead of rclcpp/rclpy. The DDS layer is replaced
by a lightweight XRCE-DDS client that communicates through the micro-ROS agent.

### rcl vs rclc
| Library | Role |
|---|---|
| `rcl` | Low-level C: owns the types (node, publisher, timer...) and raw init functions |
| `rclc` | Higher-level C: adds executor pattern + simplified `_default` init wrappers |

Types are defined in rcl; convenience inits are provided by rclc.

### Role of the Three Foundation Objects

Before creating any node, publisher, or timer, three objects must exist. Every
other entity depends on at least one of them:

| Object | Role |
|---|---|
| `rcl_allocator_t` | **Memory manager.** Wraps `malloc`/`free`/`realloc` as function pointers. Every entity that allocates heap memory (node, publisher, executor...) receives the allocator and uses it for all internal allocations. On embedded systems this lets you swap in a custom allocator (e.g. RTOS memory pool) without changing any other code. |
| `rclc_support_t` | **Session context.** Initializes the ROS2 DDS network session and stores the allocator. Think of it as the "connection to ROS2". It is passed to node init (and timer init) so they know which network session they belong to and which allocator to use. |
| `rclc_executor_t` | **Callback scheduler.** Holds a list of registered callbacks (timers, subscriptions, services). When you call `spin_some`, the executor checks which callbacks are ready (timer elapsed, message arrived, etc.) and runs them. Without the executor, callbacks are never invoked even if messages arrive. |

**Why C/C++ exposes these objects but Python does not:**

These three objects exist in every ROS2 node regardless of language — Python just
hides them automatically:

| Concern | Python (rclpy) | C (rclc) |
|---|---|---|
| Memory | Python's garbage collector manages all memory automatically — no allocator needed | C has no GC; heap must be managed explicitly. On embedded systems the heap is tiny, so the allocator lets you control or replace it |
| Context/Support | `rclpy.init()` stores the context as a hidden global inside the rclpy module | C has no module-level globals by design (unsafe in multi-threaded/embedded code). Support must be passed explicitly to every function that needs the context |
| Executor | `rclpy.spin(node)` silently creates a default executor and runs it for you | C has no default threading model. On a microcontroller you must decide *when* callbacks run (`spin_some` in `loop()`) — the executor is your explicit control point |

In short: Python hides complexity behind the language runtime and OOP. C forces
you to own every resource because there is no runtime to fall back on.

### Entity Initialization Patterns and Lifecycle

Every micro-ROS entity follows one of three initialization patterns. Understanding
why the variation exists requires knowing one C fact:

> **In C, local variables are never automatically zeroed.** When you write
> `rcl_node_t node;`, the compiler allocates stack space but leaves the bytes as
> whatever was previously stored there — old function data, random values. This is
> called *uninitialized memory*. If code reads a pointer from that memory and the
> value happens to be non-NULL (which it usually is), it looks like a valid
> (but corrupt) address. rcl exploits this: it stores an internal `impl` pointer
> that is NULL when "not yet initialized" and non-NULL when "initialized". Before
> init, rcl checks: if `impl != NULL` → refuse, assume double-init. This is the
> safety guard. The zero-init step exists solely to guarantee `impl = NULL` going in.

The three patterns and why each exists:

| Pattern | Entity | Who zeroes? | Reason for the difference |
|---|---|---|---|
| **Returned complete** | `rcl_allocator_t` | `rcl_get_default_allocator()` overwrites every field | Contains only function pointers — no `impl` pointer to check, returned fully formed by value |
| **Self-zeroing init** | `rclc_support_t` | `rclc_support_init()` calls `memset` internally | rclc is a higher-level library; it hides the zeroing as a design choice to reduce boilerplate |
| **Explicit two-step** | node, publisher, timer, subscriber, service | You, via `rcl_get_zero_initialized_*()` | rcl exposes this explicitly — you zero it first, then rcl checks `impl == NULL` before proceeding |

**Always initialize in this order** (each depends on the one above):

```c
// Pattern 1 — returned complete
rcl_allocator_t allocator = rcl_get_default_allocator();

// Pattern 2 — self-zeroing
rclc_support_t support;
rclc_support_init(&support, 0, NULL, &allocator);

// Pattern 3 — explicit two-step (node, publisher, timer, subscriber, service)
rcl_node_t node = rcl_get_zero_initialized_node();       // zero: impl = NULL
rclc_node_init_default(&node, "name", "", &support);     // init: rcl checks NULL

rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
rclc_publisher_init_default(&pub, &node, type_support, "topic");

rcl_timer_t timer = rcl_get_zero_initialized_timer();
rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(period_ms), callback);
// Note: timer takes &support, not &node
```

**Teardown — always reverse order (children before parents):**

```c
// Reverse of init order — fini children before the parent they depend on
rcl_publisher_fini(&pub, &node);          // pub depends on node
rcl_timer_fini(&timer);
rclc_executor_fini(&executor);
rcl_node_fini(&node);                     // node depends on support
rclc_support_fini(&support);
// allocator — no fini needed (stack-allocated by value, no heap)
```

Each `_fini` releases: DDS network registrations, heap memory allocated during init,
and ROS graph entries. Skipping fini causes memory leaks and ghost nodes still
visible in `ros2 node list` after your program exits.

### Executor (rclc-only — does not exist in rcl)
```c
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
rclc_executor_init(&executor, &support.context, NUM_HANDLES, &allocator);

// Register all callbacks (NUM_HANDLES must equal total count registered)
rclc_executor_add_timer(&executor, &timer);
rclc_executor_add_subscription(&executor, &sub, &msg, &cb, ON_NEW_DATA);
```

**Executor spin variants — choose based on your runtime:**

| Function | Behaviour | Blocks? | Use when |
|---|---|---|---|
| `rclc_executor_spin(&executor)` | Loops forever calling spin_some internally | Forever | Bare-metal C programs (main loop with no other work) |
| `rclc_executor_spin_some(&executor, timeout_ns)` | Checks once for ready callbacks, waits up to timeout, returns | No | **Arduino `loop()`** |
| `rclc_executor_spin_one_period(&executor, period_ns)` | Runs one check then sleeps with `usleep()` to fill the period | For one period | POSIX/Linux targets only — `usleep` may not exist on embedded |
| `rclc_executor_spin_period(&executor, period_ns)` | Like spin_one_period but loops forever | Forever | POSIX/Linux targets only |

**For Arduino, `spin_some` is the correct choice:**

Arduino's framework already calls `loop()` repeatedly in its own infinite loop.
`spin_some` checks for any ready callback, runs it, and returns — giving control
back to `loop()` so other Arduino code (sensor reads, LED, etc.) can run.

All other variants either block forever (preventing `loop()` from completing its
iteration) or call `usleep()` which may not exist on Teensy/Arduino targets.

```c
// Correct for Arduino:
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(timeout_ms));
    // other Arduino code here runs every iteration
}
```

The `timeout_ms` should be less than or equal to your fastest timer period so the
executor checks for ready work frequently enough to not miss a firing.

---

## 3. Adding a Custom Message in micro-ROS

### Why This Is Different from Standard ROS2
In standard ROS2 (rclcpp), adding a message package is a CMake dependency — the
message headers are generated at workspace build time and linked dynamically.

In micro-ROS (rclc), the **entire micro-ROS C library is pre-compiled** into a
static `.a` file at PlatformIO build time. Custom messages must be included in
that colcon build — they cannot be added after the fact.

### Step 1 — Ensure the package is valid
Your message package must have a proper `CMakeLists.txt` and `package.xml` that
follow ROS2 interface package conventions.

**Why:** The micro-ROS build script runs `colcon build` on this package inside a
cross-compilation environment that only contains the micro-ROS subset of ROS2.
Any dependency that does not exist in that environment will cause the build to fail.

**`package.xml` requirements for a pure interface package:**

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<!-- Declare every ROS2 package your .msg files reference as a <depend> -->
<!-- colcon uses these to determine build order                         -->
<depend>builtin_interfaces</depend>
<depend>std_msgs</depend>          <!-- only if your .msg uses std_msgs types -->

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

**Common mistakes:**
- `<depend>rclcpp</depend>` / `<depend>rclpy</depend>` — these are full desktop
  client libraries, not available in the micro-ROS cross-compile environment.
  Remove them.
- `<buildtool_depend>ament_cmake_python</buildtool_depend>` — not needed for
  packages that contain only `.msg` / `.srv` files. Remove it.
- Missing `<depend>std_msgs</depend>` when your `.msg` uses `std_msgs/Header` —
  colcon won't know to build `std_msgs` first, causing a CMake error:
  `Could not find a package configuration file provided by "std_msgs"`.

### Step 2 — Update platformio.ini
```ini
board_microros_user_meta = colcon.meta
```

**Why `colcon.meta`:** Tells the build script to merge your custom colcon
configuration into the board's default meta before running `colcon build`.

**Note:** `board_microros_extra_packages_path` is not a supported option — it is
silently ignored by the build script. The package discovery path is hardcoded to
`extra_packages/` inside the PlatformIO project directory (see Step 3).

### Step 3 — Create `extra_packages/` and add your package
The build script always looks for custom packages in a folder named `extra_packages`
at the root of the PlatformIO project (same level as `platformio.ini`). Create it
and add a symlink to your package:

```bash
mkdir -p <pio_project_root>/extra_packages
ln -s /absolute/path/to/your_package \
      <pio_project_root>/extra_packages/your_package
```

**Why symlink instead of copy:** The package source stays in your ROS2 workspace
where it can be built by both the host-side agent (`colcon build`) and the
micro-ROS firmware build. A copy would require manual sync every time the `.msg`
files change.

### Step 4 — Create colcon.meta in the PlatformIO project root
```json
{
    "names": {
        "micro_ros_msgs": {},
        "<your_package>": {}
    }
}
```

**Why `micro_ros_msgs` is listed:** It is an internal micro-ROS package already
present in the default source tree. Listing it here with `{}` is optional but
harmless — it simply confirms default CMake settings with no overrides.
**Why empty `{}`:** Use default CMake settings. You can add cmake args here if needed
(e.g. `{"cmake-args": ["-DBUILD_TESTING=OFF"]}`).

### Step 5 — Clean and rebuild the micro-ROS library

**Which `pio` to use:**
PlatformIO can be installed in two places: system-wide (`pip install platformio`)
and inside its own managed Python environment (`~/.platformio/penv/bin/pio`). The
VS Code PlatformIO extension always uses the managed environment. Mixing the two
on the same project causes divergent build caches and inconsistent results.

Rule: use the same `pio` binary for every command on a project. The safest default
is always the managed-environment one:

```bash
~/.platformio/penv/bin/pio run -t clean_microros   # delete old compiled library
~/.platformio/penv/bin/pio run                      # rebuild with custom package
```

**Why clean first:** PlatformIO's incremental build detects no `.cpp` source changes
and skips rebuilding the micro-ROS library, silently ignoring your new `colcon.meta`.
`clean_microros` deletes the `.a` library and all generated headers, forcing a full rebuild.

### Step 6 — Use the message in code

**C naming rules (ROS2 → C):**

ROS2 uses CamelCase package and message names. C flattens these with double
underscores and snake_case headers:

| ROS2 | C equivalent |
|---|---|
| Package `<your_package>` | Prefix `<your_package>__msg__` |
| Message `<YourMsg>` (CamelCase) | Type `<your_package>__msg__<YourMsg>` |
| Include path | `<your_package>/msg/<your_msg>.h` (snake_case filename) |
| Type support | `ROSIDL_GET_MSG_TYPE_SUPPORT(<your_package>, msg, <YourMsg>)` |

```c
#include <<your_package>/msg/<your_msg>.h>

<your_package>__msg__<YourMsg> my_msg;

rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(<your_package>, msg, <YourMsg>),
    "/topic_name"
);
```

**Handling string fields (e.g. `std_msgs/Header.frame_id`):**

Strings in micro-ROS C use `rosidl_runtime_c__String` — a struct holding a
`data` pointer, `size`, and `capacity`. Unlike C++, you cannot assign a string
literal directly; you must point the struct at a char buffer.

```c
// Do once in setup() — static ensures the buffer lives for the program lifetime
static char frame_id_buf[] = "my_frame";
my_msg.header.frame_id.data     = frame_id_buf;
my_msg.header.frame_id.size     = strlen(frame_id_buf);
my_msg.header.frame_id.capacity = sizeof(frame_id_buf);
```
