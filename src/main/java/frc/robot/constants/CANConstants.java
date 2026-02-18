package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class CANConstants {

  public static final int frontLeftDrive = 2;
  public static final int frontRightDrive = 8;
  public static final int backLeftDrive = 6;
  public static final int backRightDrive = 9;

  public static final int frontLeftRotation = 1;
  public static final int frontRightRotation = 3;
  public static final int backLeftRotation = 5;
  public static final int backRightRotation = 10;

  public static final int swerveFrontLeft = 2;
  public static final int swerveFrontRight = 3;
  public static final int swerveBackLeft = 0;
  public static final int swerveBackRight = 4;

  public static final int turretEncoder = 10;
  public static final int turretMotor = 11;

  public static final int hoodMotor = 0; // TODO: Set the real id once we have it
  public static final int intakeMotor = 0;
  public static final int flyWheelMotor = 0;

  public static CANBus driveBaseCanivore = new CANBus("DriveBase");
}
