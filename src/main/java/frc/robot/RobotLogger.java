package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotLogger {
  private static final String SEPARATOR = "/";
  private static boolean enabled = true;

  static {
    DogLog.setEnabled(enabled);
  }

  // Prefix for logs to categorize data easily
  private final String subsystemName;

  public RobotLogger(String subsystemName) {
    this.subsystemName = subsystemName;
  }

  public static void setEnabled(boolean newEnabled) {
    enabled = newEnabled;
    DogLog.setEnabled(newEnabled);
  }

  public static boolean isEnabled() {
    return enabled;
  }

  public void log(String key, double value) {
    if (enabled) {
      DogLog.log(subsystemName + SEPARATOR + key, value);
    }
  }

  public void logAndDisplay(String key, double value) {
    log(key, value);
    // Also display on SmartDashboard for real-time monitoring
    SmartDashboard.putNumber(subsystemName + SEPARATOR + key, value);
  }

  public void log(String key, boolean value) {
    if (enabled) {
      DogLog.log(subsystemName + SEPARATOR + key, value);
    }
  }

  public void logAndDisplay(String key, boolean value) {
    log(key, value);
    SmartDashboard.putBoolean(subsystemName + SEPARATOR + key, value);
  }

  public void log(String key, String value) {
    if (enabled) {
      DogLog.log(subsystemName + SEPARATOR + key, value);
    }
  }

  public void logAndDisplay(String key, String value) {
    log(key, value);
    SmartDashboard.putString(subsystemName + SEPARATOR + key, value);
  }

  public <T extends Enum<T>> void log(String key, T value) {
    if (enabled) {
      DogLog.log(subsystemName + SEPARATOR + key, value.name());
    }
  }

  public <T extends Enum<T>> void logAndDisplay(String key, T value) {
    log(key, value);
    SmartDashboard.putString(subsystemName + SEPARATOR + key, value.name());
  }

  public void log(String key, Pose2d pose) {
    if (enabled) {
      DogLog.log(subsystemName + SEPARATOR + key, pose);
    }
  }

  public void log(String key, Rotation2d rotation) {
    if (enabled) {
      DogLog.log(subsystemName + SEPARATOR + key, rotation);
    }
  }

  public void log(String key, Translation2d translation) {
    if (enabled) {
      DogLog.log(subsystemName + SEPARATOR + key, translation);
    }
  }
}