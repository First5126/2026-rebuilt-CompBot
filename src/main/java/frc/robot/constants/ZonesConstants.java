package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public class ZonesConstants {
  public interface RectangularRegion {
    Translation2d getTopLeftTranslation();

    Translation2d getBottomRightTranslation();
  }

  public enum Zone implements RectangularRegion {
    ALLIANCE_ZONE(new Translation2d(0.0, 0.0), new Translation2d(4.0, 8.0)),
    NEUTRAL_ZONE_RIGHT(new Translation2d(4.0, 0.0), new Translation2d(12.0, 4.0)),
    NEUTRAL_ZONE_LEFT(new Translation2d(4.0, 4.0), new Translation2d(12.0, 8.0)),
    OPPONENT_ZONE(new Translation2d(12.5, 0.0), new Translation2d(16.5, 8.0)),
    OUT_OF_BOUNDS(null, null);

    private final Translation2d topLeftTranslation;
    private final Translation2d bottomRightTranslation;

    Zone(Translation2d topLeftTranslation, Translation2d bottomRightTranslation) {
      this.topLeftTranslation = topLeftTranslation;
      this.bottomRightTranslation = bottomRightTranslation;
    }

    public Translation2d getTopLeftTranslation() {
      return topLeftTranslation;
    }

    public Translation2d getBottomRightTranslation() {
      return bottomRightTranslation;
    }
  }

  public static final Angle BUMP_ANGLE = Degrees.of(5.0);

  public enum Trench implements RectangularRegion {
    BLUE_OUTPOST_SIDE(new Translation2d(4, 1.25), new Translation2d(5.25, 0)),
    BLUE_DEPOT_SIDE(new Translation2d(4, 6.75), new Translation2d(5.25, 8)),
    RED_OUTPOST_SIDE(new Translation2d(11.5, 1.25), new Translation2d(12.5, 0)),
    RED_DEPOT_SIDE(new Translation2d(11.5, 6.5), new Translation2d(12.5, 8));


    private final Translation2d topLeftTranslation;
    private final Translation2d bottomRightTranslation;

    Trench(Translation2d topLeftTranslation, Translation2d bottomRightTranslation) {
      this.topLeftTranslation = topLeftTranslation;
      this.bottomRightTranslation = bottomRightTranslation;
    }

    public Translation2d getTopLeftTranslation() {
      return topLeftTranslation;
    }

    public Translation2d getBottomRightTranslation() {
      return bottomRightTranslation;
    }
  }

  public static enum HubDeadZone implements RectangularRegion {
    BLUE_HUB_DEADZONE(new Translation2d(5.203, 4.595), new Translation2d(6.258, 3.556)),
    RED_HUB_DEADZONE(new Translation2d(10.379, 4.595), new Translation2d(11.223, 3.459));

    private final Translation2d topLeftTranslation;
    private final Translation2d bottomRightTranslation;

    HubDeadZone(Translation2d topLeftTranslation, Translation2d bottomRightTranslation) {
      this.topLeftTranslation = topLeftTranslation;
      this.bottomRightTranslation = bottomRightTranslation;
    }

    public Translation2d getTopLeftTranslation() {
      return topLeftTranslation;
    }

    public Translation2d getBottomRightTranslation() {
      return bottomRightTranslation;
    }
  }

  public static boolean contains(Translation2d position, RectangularRegion region) {
    if (position == null || region == null) {
      return false;
    }
    Translation2d corner1 = region.getTopLeftTranslation();
    Translation2d corner2 = region.getBottomRightTranslation();
    if (corner1 == null || corner2 == null) {
      return false;
    }

    double minX = Math.min(corner1.getX(), corner2.getX());
    double maxX = Math.max(corner1.getX(), corner2.getX());
    double minY = Math.min(corner1.getY(), corner2.getY());
    double maxY = Math.max(corner1.getY(), corner2.getY());

    double x = position.getX();
    double y = position.getY();
    return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
  }

  public static <E extends Enum<E> & RectangularRegion> boolean containsAny(
      Translation2d position, Class<E> enumClass) {
    if (position == null || enumClass == null) {
      return false;
    }
    E[] values = enumClass.getEnumConstants();
    if (values == null) {
      return false;
    }
    for (E value : values) {
      if (contains(position, value)) {
        return true;
      }
    }
    return false;
  }

  public static <E extends Enum<E> & RectangularRegion> E firstContainingOrDefault(
      Translation2d position, Class<E> enumClass, E defaultValue) {
    if (position == null || enumClass == null) {
      return defaultValue;
    }
    E[] values = enumClass.getEnumConstants();
    if (values == null) {
      return defaultValue;
    }
    for (E value : values) {
      if (contains(position, value)) {
        return value;
      }
    }
    return defaultValue;
  }
}
