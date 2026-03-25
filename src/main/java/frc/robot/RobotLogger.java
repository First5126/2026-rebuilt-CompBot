package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotLogger {
  private static final String SEPERATOR = "/";
  public static final boolean ENABLE_LOGGING = false; // Set to false to disable all logging
  // Prefix for logs to categorize data easily
  private final String subsystemName;

  public RobotLogger(String subsystemName) {
    this.subsystemName = subsystemName;
  }

  public void log(String key, double value) {
    // Log to DogLog with a structured key
    String fullKey = subsystemName + SEPERATOR + key;
    if (ENABLE_LOGGING) {
      DogLog.log(fullKey, value);
    }
  }

  public void logAndDisplay(String key, double value) {
    log(key, value);
    // Also display on SmartDashboard for real-time monitoring
    SmartDashboard.putNumber(subsystemName + SEPERATOR + key, value);
  }

  public void log(String key, boolean value) {
    String fullKey = subsystemName + SEPERATOR + key;

    if (ENABLE_LOGGING) {
      DogLog.log(fullKey, value);
    }
  }

  public void logAndDisplay(String key, boolean value) {
    log(key, value);
    SmartDashboard.putBoolean(subsystemName + SEPERATOR + key, value);
  }

  public void log(String key, String value) {
    String fullKey = subsystemName + SEPERATOR + key;
    if (ENABLE_LOGGING) {
      DogLog.log(fullKey, value);
    }
  }

  public void logAndDisplay(String key, String value) {
    log(key, value);
    SmartDashboard.putString(subsystemName + SEPERATOR + key, value);
  }

  public <T extends Enum<T>> void log(String key, T value) {
    String fullKey = subsystemName + SEPERATOR + key;
    if (ENABLE_LOGGING) {
      DogLog.log(fullKey, value.name());
    }
  }

  public <T extends Enum<T>> void logAndDisplay(String key, T value) {
    log(key, value);
    SmartDashboard.putString(subsystemName + SEPERATOR + key, value.name());
  }

  public void log(String key, Pose2d pose) {
    if (ENABLE_LOGGING) {
      DogLog.log(key, pose);
    }
  }
}
