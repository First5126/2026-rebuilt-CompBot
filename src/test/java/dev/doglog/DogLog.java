package dev.doglog;

import edu.wpi.first.math.geometry.Pose2d;

/** Test stub to avoid requiring DogLog runtime initialization. */
public final class DogLog {
  private DogLog() {}

  public static void log(String key, double value) {}

  public static void log(String key, boolean value) {}

  public static void log(String key, String value) {}

  public static void log(String key, Pose2d pose) {}
}
