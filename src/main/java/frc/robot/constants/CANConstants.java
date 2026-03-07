package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class CANConstants {

  // Swerve Drive Motors
  public static final int frontLeftDrive = 13;
  public static final int frontRightDrive = 4;
  public static final int backLeftDrive = 11;
  public static final int backRightDrive = 6;

  // Swerve Rotation Motors
  public static final int frontLeftRotation = 15;
  public static final int frontRightRotation = 2;
  public static final int backLeftRotation = 9;
  public static final int backRightRotation = 8;

  // Swerve Motor Encoders
  public static final int swerveFrontLeftEncoder = 14;
  public static final int swerveFrontRightEncoder = 3;
  public static final int swerveBackLeftEncoder = 10;
  public static final int swerveBackRightEncoder = 7;

  // Motors
  // Intake
  public static final int intakePivotLeftMotor = 2;
  public static final int intakePivotRightMotor = 6;
  public static final int intakeWheelsMotor = 5;

  // Turret
  public static final int turretMotor = 8;
  
  public static final int rightClimberMotor = 2;
  public static final int spindexerMotor = 3;
  public static final int indexFeederMotor = 7;
  public static final int hoodMotor = 11;
  public static final int leftClimberMotor = 12;
  public static final int flyWheelsMotor = 12;

  // Encoders
  public static final int intakeEncoder = 6;
  public static final int turretEncoder = 10;
  public static final int hoodEncoder = 13;

  // Other
  public static final int pigeon = 1;
  public static final int candle = 1;
  public static final int canID = 10;

  // CANBuses
  public static CANBus driveBaseCanivore = new CANBus("DriveBase");
  public static CANBus Canivore2 = new CANBus("Canivore 2");
}
