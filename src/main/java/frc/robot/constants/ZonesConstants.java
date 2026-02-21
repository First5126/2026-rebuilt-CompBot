package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class ZonesConstants {
  public static enum Zone {
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

  public static enum Bump {
    BLUE_OUTPOST_SIDE(new Translation2d(4.07, 3.43), new Translation2d(5.19, 1.58)),
    BLUE_DEPOT_SIDE(new Translation2d(4.07, 6.47), new Translation2d(5.19, 4.62)),
    RED_OUTPOST_SIDE(new Translation2d(11.35, 3.43), new Translation2d(12.47, 1.58)),
    RED_DEPOT_SIDE(new Translation2d(11.35, 6.47), new Translation2d(12.47, 4.62));

    private final Translation2d topLeftTranslation;
    private final Translation2d bottomRightTranslation;

    Bump(Translation2d topLeftTranslation, Translation2d bottomRightTranslation) {
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
}
