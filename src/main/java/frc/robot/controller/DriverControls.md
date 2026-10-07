# Driver Controls

Summary: mappings for the Driver controller (port 0). Drive input uses a "gas pedal" scheme: the triggers set speed, the left stick sets direction.

- Right Trigger: Field-centric drive - hold to move relative to the field heading (full speed). Left stick sets direction.
- Left Trigger: Robot-centric drive - hold to move relative to the robot heading (half speed). Left stick sets direction.
- Left Stick: Movement direction. Direction comes from the stick; speed comes from whichever trigger is held.
- Right Stick X: Rotation (capped at half the max angular rate).
- Left Bumper: Reset field-centric heading (seedFieldCentric) - re-zeros forward.
- A / B / X / Y / Start / Back / D-Pad / Right Bumper / stick clicks: unused.

Notes:
- If both triggers are held, the right (field-centric) trigger takes priority.
- With no trigger and no rotation input, the drivetrain brakes.
- Deadbands are configured in code (see ControllerConstants / CustomXboxController).

# Operator Controls

Summary: mappings for the Operator controller (port 1). All button bindings are state-independent in the current code. The Operator state (NORMAL vs OVERRIDE) only affects the turret-tracking default command: in NORMAL the turret tracks the shooting solution; in OVERRIDE the turret does nothing.

Operator state:
- NORMAL: default - turret tracking enabled.
- OVERRIDE: manual/test mode - turret tracking disabled.
- Back toggles the state and updates the SmartDashboard key `Operator OVERRIDE Active` (boolean).

Mappings:
- A (while held): set hood to trench position (0 degrees)
- Y (while held): stop flywheel
- B (on press): start indexer
- B (on release): clear shooting jam - flywheel inward + indexer reverse for 0.25 s, then stop indexer
- X (on press): clear shooting jam - flywheel inward + indexer reverse
- Right Bumper (on press): lower intake pivot
- Right Trigger (on press / on release): intake wheels in / stop
- Left Trigger (on press / on release): intake wheels out / stop
- D-Pad Left (on press): turret offset +1 degree
- D-Pad Right (on press): turret offset -1 degree
- Back (on press): toggle Operator state (NORMAL <-> OVERRIDE)
- Start: unused
- D-Pad Up / Down: unused
- Left / Right sticks: unused

Default commands (RobotContainer):
- ShootingMechanism: turret tracking (NORMAL state only)
- FlyWheel: flywheel + hood set to the current shooting solution
- When the robot is near the trench, the hood is commanded to the trench position.

-- End of controls documentation.