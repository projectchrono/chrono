Time Synchronization and Sim-to-Real Best Practices {#ros_sim_to_real}
===================================================

Chrono::ROS is designed so that the same ROS 2 control and perception stack can
run against a Chrono simulation and against real hardware with no code changes.
This page covers the time model the bridge uses, how to design ROS nodes that
behave predictably against it, how to keep a stack Sim-to-Real Ready, and how 
to coordinate multiple simulations sharing one ROS graph.

## Simulation time

The simulation is the master clock. The `ChROSClockHandler` publishes the current
simulation time as `rosgraph_msgs/msg/Clock` on `/clock`, and every handler stamps
its messages' `header.stamp` with simulation time. ROS nodes started with the
parameter **`use_sim_time:=true`** take `/clock` as their time source instead of
the wall clock, so their timers, message stamps, and tf lookups all run on
simulation time.

Chrono advances simulation time with a **fixed timestep**: each
`sys.DoStepDynamics(Δt)` steps the clock by a constant Δt. That simulation time is
decoupled from wall-clock time — the loop runs as fast as it can, or is throttled
to real time (a **real-time factor** of 1) with `ChRealtimeStepTimer`. This
fixed-step, real-time-factor model is the standard physics/robotics simulation
convention (e.g. Gazebo's max-step-size and real-time-factor).

Two things are required:

- Register a `ChROSClockHandler` on the manager, so `/clock` is published.
- Set `use_sim_time:=true` on every ROS node in the graph.

If a node sets `use_sim_time:=true` but no `/clock` is published, it blocks
waiting for time. If it leaves `use_sim_time:=false` while the simulation does not
run in real time, it desynchronizes from the data.

## Designing ROS nodes for predictable results

- **Use the ROS clock, not the wall clock.** Read time with `node->now()` /
  `node->get_clock()->now()`, and create periodic callbacks with a ROS-clock timer
  (`rclcpp::create_timer(node, node->get_clock(), period, callback)`), **not**
  `create_wall_timer()`. `create_wall_timer()` always uses the steady (wall) clock
  and ignores `use_sim_time`, so a control loop built on it runs at wall-clock rate
  regardless of simulation time — the most common source of sim/real timing
  mismatch.
- **Subscription callbacks are data-driven.** A callback fires when a message
  arrives, so in simulation it is paced by the rate at which the bridge publishes
  (a simulation-time rate). Beyond the clock points above, event-driven nodes need
  no special handling.
- **Reproducibility.** With your controller in a separate ROS node, the closed loop is
  reproducible at the *behavioral* level — the same scenario produces the same overall
  behavior — which is what system tuning, validation, and hardware-in-the-loop need. It
  is not always *bit*-reproducible run to run: the simulation does not block on ROS, so
  OS scheduling and transport jitter shift a command's arrival by a step or two between
  runs. Your controller's own logic might be deterministic; only the cross-process timing 
  is not, and this is inherent to ROS co-simulation in general — not specific to
  Chrono::ROS, whose time integration is itself deterministic. If you need *bit-exact*
  reproducibility (e.g. regression baselines), either gate stepping on the controller with `ChROSSubscription::WaitForMessage(timeout)` (the "synchronous mode" pattern), or run 
  the control logic in-process as a C++ `ChROSHandler` tied to Chrono rather than in a 
  separate node.

## Sim-to-Real: one stack for simulation and hardware

A control or perception stack moves between Chrono and a real robot unchanged when:

- **Topics, message types, and QoS are identical** in both. Chrono::ROS publishes
  standard messages (`sensor_msgs/Image`, `sensor_msgs/NavSatFix`, …), so the stack
  cannot tell the source apart.
- **Time comes from the ROS clock.** Use `node->now()` and ROS-clock timers (above),
  then run with `use_sim_time:=true` in simulation and `use_sim_time:=false` on
  hardware — the same node code works in both.
- **Account for compute latency (causality).** On hardware, the control loop and any
  CV inference consume real (wall) time while the world keeps moving; that latency is
  part of the system. Two ways to reproduce it in simulation:
  - *Real-time-paced (causal, realistic):* pace the simulation at, or below, real time
    (`ChRealtimeStepTimer`). A 50 ms inference then costs 50 ms of simulated motion,
    as on hardware. If the control or perception cannot keep up at real time, slow the
    simulation below real time rather than letting it run ahead, so the latency stays
    proportionate.
  - *Lock-stepped (deterministic, optimistic):* gating with `WaitForMessage` freezes
    the world while the controller computes, so its wall-clock latency is not reflected
    in simulation time. This is ideal for reproducibility but hides real compute
    latency — use it for training and regression, not for latency-sensitive sim-to-real
    validation.

  Budget your control-loop solve time and inference time, and choose the mode that
  matches your goal.

## Lock-step (synchronous) stepping

By default the simulation never blocks on ROS. To make the closed loop
bit-reproducible, gate each step on the controller's reply:
`ChROSSubscription::WaitForMessage` blocks the loop until a message arrives (or a
timeout elapses), so the world advances only after the command is in hand:

~~~{.cpp}
while (...) {
    publish_state();                  // handlers, via manager->Update() or directly
    sub->WaitForMessage(timeout);     // block until the controller responds
    sys.DoStepDynamics(step);         // only now advance the world
}
~~~

See the Reproducibility and Sim-to-Real notes above for when this mode fits
(regression and training) and what it costs (hidden compute latency).

## Multiple simulations in one ROS graph and Multiple Simulated Agents

Several Chrono simulations can share a ROS graph and be coordinated through it. The
building blocks are already present; the orchestration is left to the application.

- **One clock authority.** Only one participant may publish `/clock`. Register
  `ChROSClockHandler` on the master simulation (or a dedicated clock node) and **do
  not register it on the others**. A Chrono manager publishes nothing on its own, so
  a follower simply omits that handler.
- **Followers track the master clock.** A follower subscribes to `/clock` and advances
  its own `DoStepDynamics` only up to the received time, blocking with `WaitForMessage`
  on that subscription — the reverse of the usual publish direction. This keeps the
  simulations aligned in time.
- **Shared `/tf`.** `/tf` is a multi-publisher topic by convention: each participant
  publishes only its own transforms and tf2 aggregates them into one tree keyed by
  frame pair, so nothing is overwritten. Give each simulation **unique frame ids**
  (for example, namespaced `robot1/base_link`) so the trees do not collide.

This coordinates *time*. Sharing a *world* additionally requires each simulation to
publish its agents' states and consume the others' (as proxy bodies), and strict
barrier lock-step requires an all-participants-ready handshake. For large-scale
distributed multi-agent co-simulation, Chrono::SynChrono is the purpose-built tool;
the pattern here suits a few loosely-coupled simulations sharing a ROS graph.
