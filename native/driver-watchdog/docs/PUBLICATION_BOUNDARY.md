# Publication boundary

Included: the two newly authored watchdog/serial helper headers; the newly
authored independent PTY test (only include paths adapted for this standalone
layout); standalone CMake and these project-authored instructions.

Excluded: manufacturer ychd_robot.cpp/ychd_robot.h, Quaternion_Solution sources,
package.xml, CMake/launch/config/udev copies, installed driver binaries, firmware,
private deployment scripts/paths, maps, trajectories, authorization records,
serial dumps, field logs and diagnostic evidence. The development driver's full
patch also contains removed manufacturer implementations and is not included.

The existing repository DEPENDENCIES.md records manufacturer
turn_on_ychd_robot 0.0.0 as `TODO: License declaration`. The owner's permission
to publish project-authored changes is not used to assign a license to that
package. Pvpproof-owned project components are licensed under MIT (see root LICENSE).
This does not relicense manufacturer materials or claim complete third-party compatibility.
Future redistribution of a vendor-specific contextual patch or full driver
requires resolving that package's applicable terms separately.
