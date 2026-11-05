# Integration contract for a separately supplied chassis driver

This document describes the project-authored integration. It does not contain or
license the manufacturer driver. The inspected manufacturer package metadata
still states `TODO: License declaration`; the existing repository third-party
review therefore excludes its source and binary. Obtain a separately authorized
local driver. Do not apply a context-heavy vendor patch from development copies
as though it contained only project code.

## Required wiring

1. Add these two headers to the separately supplied driver's build and link
   pthreads. No ROS sources or manufacturer headers are included in this bundle.
2. Create one `charging::CommandWatchdog` with a new UUID at each driver startup.
   Never restore ready/unlatched state from disk. Retain the independent arbiter
   gate; only that arbiter may publish the driver's motion command input.
3. Replace the command callback's former direct serial write with
   `watchdog.command(twist.linear.x, twist.linear.y, twist.angular.z)`.
   Subscribe with `rclcpp::QoS(rclcpp::KeepLast(1))`. Disable other serial command
   endpoints/bypasses. Merely adding a watchdog while leaving old writers active
   does not establish a single control path.
4. After each successfully parsed and validated raw chassis feedback frame,
   call `watchdog.feedback(vx, vy, wz)` using its raw velocity values. A read
   returning no byte is not a feedback sample. Do not substitute fused/map data.
5. After the local RX serial endpoint is configured, open `BoundedSerialTx` for
   that same endpoint and start one dedicated steady-clock thread. Every 20 ms
   call `watchdog.transmit([&](const CommandWatchdog::Frame &f) { return tx.write(f); })`.
   That is the only active TX writer. A ROS timer serviced behind a blocking
   serial read is insufficient. Never hold an RX mutex in this thread, and do
   not add drain/flush or blocking ROS publication to it.
6. Publish `watchdog.state()` from the ROS executor every 50 ms on
   `/ychd_robot/command_watchdog_state` (`std_msgs/String`, volatile depth 1).
   Declare read-only `charging_return_driver_gate_version=2`.
7. Expose `/ychd_robot/reset_command_watchdog` (`std_srvs/Trigger`). Its only
   state mutation is `response.message=watchdog.reset(response.success)`.
   Ordinary Twist, reconnection, fresh messages and GUI restart must not reset it.
8. Stop and join the TX worker before final zero transmission and closing the
   endpoint. No writer may race shutdown. Startup failures must remain stopped.

`CommandWatchdog` holds its state mutex during the bounded writer invocation:
no nonzero frame selected earlier can be emitted after a reset/fault ACK.
`BoundedSerialTx` performs an independent O_NONBLOCK write with a 20 ms deadline;
its second descriptor neither configures termios nor accesses the RX mutex.
Actual tty dual-open compatibility and endpoint identity require parked local
validation; the PTY test establishes only software behavior.

## Reset/status handshake

The JSON fields are `version` (2), `session` (new UUID), `state_seq` (increasing),
`monotonic_ns` (producer steady-clock ns), `latched`, `reason`, `event`,
`command_age_s` and `feedback_age_s`. Before initial data, ages are finite -1.
Events: `state`, `reset_ack`, `reset_rejected`. Reasons: startup_reset_required,
ready, command_timeout, feedback_timeout, invalid_command, serial_tx_failed.
The first fault reason is retained until explicit successful reset.

Reset succeeds only with a zero command <=250 ms old, valid raw feedback
<=250 ms old and continuously quiet >=2 s (|vx|/|vy| <=.01 m/s, |wz| <=.02 rad/s),
and a successful TX <=100 ms old. Feedback gaps invalidate the quiet interval.

The arbiter must separate static contract readiness from motion readiness so an
explicit parked reset remains possible while the driver is latched. Continue
zero streaming during asynchronous reset. Require service success, same session,
fresh source/receipt times and `latched=false`, followed by a newer topic state
with `state_seq` greater than the ACK, before issuing its own final GUI reset ACK.
Session changes invalidate pending reset and run authorization. Reset cannot
start or resume a prior run. Compare monotonic source time only between processes
on the same Linux host, not between machines.

## Validation boundaries

The standalone test covers startup latch, explicit reset, command and feedback
loss, recovery without implicit resume, invalid commands, TX failure/backpressure,
frame integrity, and a blocked 2 s receive path with an independent TX thread.
ROS integration separately needs tests for queue depth, stale/changed sessions,
reset ACK races, cancel/stop, command ownership and frozen/lost GUI leases.

250 ms is an expiry threshold, not a certified stop deadline: the 20 ms worker
cycle, bounded write attempt and non-real-time scheduling add latency. A write
success means kernel acceptance, not MCU acknowledgement. An in-process worker
cannot act after its entire process or host dies; an independent controller
command timeout is not supplied or proven by this bundle. Recorded paths do not
provide rear obstacle detection. Field acceptance and actual stop distance must
be verified separately by the responsible operator.
