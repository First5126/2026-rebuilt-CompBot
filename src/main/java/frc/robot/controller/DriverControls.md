# Driver Controls

Summary: mappings for the Driver controller. Actions are executed directly (no Operator state switching).

- Left Trigger: Drive Robot-Centric (hold to drive relative to robot heading)
- Right Trigger: Drive Field-Centric (hold to drive relative to field heading)
- Left Bumper: Reset zero (zero sensors / heading)
- Right Stick (analog): Rotation control (rotate robot with right stick)
- Left Stick Click: Movement (translation with left stick)
- D-Pad / A / B / X / Y / Start / Back / Right Bumper / Right Stick Click: unused / no mapping

Notes:
- Driver mappings are immediate control inputs used for robot motion only.
- Joystick deadbands and controller constants are configured in code (see ControllerConstants).

# Operator Controls

Summary: mappings for the Operator controller. Many bindings are state-dependent (NORMAL vs OVERRIDE). Toggle operator state with Back button. Start button resets match-time calibration.

Operator state behavior:
- NORMAL: primary automated/operator-assisted behaviors.
- OVERRIDE: manual / direct control mode for testing or field adjustments.
- Back button toggles state and updates SmartDashboard key: `Operator OVERRIDE Active` (boolean).

Common implementation details:
- Deadband triggers created for left/right sticks at 0.10.
- Several buttons use SelectCommand to choose a command depending on OperatorState.
- `CommandFactory` must be provided to Operator (init) before configureBindings is called.

Mappings (state-specific where applicable):

- A
  - On press (onTrue):
  - NORMAL: lowerIntake()
  - OVERRIDE: startIndexer()
  - On release (onFalse):
  - NORMAL: raiseIntake()
  - OVERRIDE: stopIndexer()
  - While held (whileTrue):
  - NORMAL: setHoodToTrenchPosition()
    - OVERRIDE: none

- B
  - On press:
  - NORMAL: startIndexer()
  - OVERRIDE: startFlywheelWithSolution()
  - On release:
  - NORMAL: stopIndexer()
  - OVERRIDE: stopFlywheel()

- X
  - On press:
    - NORMAL: none
    - OVERRIDE: clearShootingJam()
  - On release:
    - NORMAL: none
    - OVERRIDE: stopFlywheelAndIndexer()

- Y
  - On press: no-op in both states

- D-Pad Up / Down
  - While held:
    - NORMAL: none
    - OVERRIDE: raiseHoodSlowly() (manual hood movement)

- D-Pad Left / Right
  - While held:
    - NORMAL: none
    - OVERRIDE: rotateTurretBy(Degrees.of(0.1)) (manual turret movement)

- Left Bumper
  - On press:
    - NORMAL: raiseIntake()
    - OVERRIDE: raiseIntake()

- Right Bumper
  - On press:
    - NORMAL: lowerIntake()
    - OVERRIDE: lowerIntake()

- Right Trigger
  - On press:
    - NORMAL: none
    - OVERRIDE: reverseIntake()
  - On release:
    - NORMAL: none
    - OVERRIDE: stopIntake()

- Left Trigger
  - On press:
    - NORMAL: none
    - OVERRIDE: startIntake()
  - On release:
    - NORMAL: none
    - OVERRIDE: stopIntake()

- Left stick X (deadband > 0.10)
  - While held: rotateTurretWithStickInput(this::getLeftX) — manual turret with analogue input

- Right stick Y (deadband > 0.10)
  - While held: rotateHoodWithStickInput(this::getRightY) — manual hood with analogue input

- Start
  - On press: reset match-time calibration (ShiftData.resetMatchTimeCalibration())

- Back
  - On press: toggle Operator state (NORMAL <-> OVERRIDE) and update SmartDashboard key `Operator OVERRIDE Active`

Operator OVERRIDE template (manual mode quick reference)
- Left Trigger: Intake wheels in
- Right Trigger: Intake wheels out
- Left Bumper: Intake pivot in
- Right Bumper: Intake pivot out
- D-Pad Up: Hood up (manual)
- D-Pad Down: Hood down (manual)
- D-Pad Left: Turret left (manual)
- D-Pad Right: Turret right (manual)
- B: Flywheel out (interpolated / start flywheels with solution)
- A: Spin indexer / spindexer (indexing while held)
- X: Stop/Everything in (static speed) — mapped to reverse/stop shooting in code

-- End of controls documentation.
