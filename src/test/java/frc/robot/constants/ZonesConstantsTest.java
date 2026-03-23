package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ZonesConstantsTest {

  @Test
  void contains_returnsFalseForNullInputs() {
    assertFalse(ZonesConstants.contains(null, ZonesConstants.Zone.BLUE_ZONE));
    assertFalse(ZonesConstants.contains(new Translation2d(1, 1), null));
    assertFalse(ZonesConstants.contains(null, null));
  }

  @Test
  void contains_returnsFalseForNullCorners() {
    assertFalse(
        ZonesConstants.contains(new Translation2d(1, 1), ZonesConstants.Zone.OUT_OF_BOUNDS));
  }

  @Test
  void contains_isInclusiveOnEdges() {
    assertTrue(ZonesConstants.contains(new Translation2d(0.0, 0.0), ZonesConstants.Zone.BLUE_ZONE));
    assertTrue(ZonesConstants.contains(new Translation2d(4.0, 8.0), ZonesConstants.Zone.BLUE_ZONE));
    assertTrue(
        ZonesConstants.contains(
            new Translation2d(12.0, 4.0), ZonesConstants.Zone.NEUTRAL_ZONE_RIGHT));
  }

  @Test
  void contains_handlesSwappedCornerOrdering() {
    // BLUE_OUTPOST_SIDE is defined with Y decreasing from topLeft->bottomRight (1.25 -> 0).
    Translation2d inside = new Translation2d(4.5, 0.5);
    assertTrue(ZonesConstants.contains(inside, ZonesConstants.Trench.BLUE_OUTPOST_SIDE));

    Translation2d outside = new Translation2d(3.9, 0.5);
    assertFalse(ZonesConstants.contains(outside, ZonesConstants.Trench.BLUE_OUTPOST_SIDE));
  }

  @Test
  void containsAny_worksForEnums() {
    assertTrue(ZonesConstants.containsAny(new Translation2d(1.0, 1.0), ZonesConstants.Zone.class));
    assertFalse(
        ZonesConstants.containsAny(new Translation2d(-1.0, -1.0), ZonesConstants.Zone.class));

    assertTrue(
        ZonesConstants.containsAny(new Translation2d(11.75, 0.5), ZonesConstants.Trench.class));
    assertFalse(
        ZonesConstants.containsAny(new Translation2d(8.0, 8.0), ZonesConstants.Trench.class));
  }

  @Test
  void firstContainingOrDefault_returnsMatchOrDefault() {
    ZonesConstants.Zone zone =
        ZonesConstants.firstContainingOrDefault(
            new Translation2d(13.0, 2.0),
            ZonesConstants.Zone.class,
            ZonesConstants.Zone.OUT_OF_BOUNDS);

    assertEquals(ZonesConstants.Zone.RED_ZONE, zone);

    ZonesConstants.Zone missing =
        ZonesConstants.firstContainingOrDefault(
            new Translation2d(-10.0, 2.0),
            ZonesConstants.Zone.class,
            ZonesConstants.Zone.OUT_OF_BOUNDS);
    assertEquals(ZonesConstants.Zone.OUT_OF_BOUNDS, missing);

    assertEquals(
        ZonesConstants.Zone.OUT_OF_BOUNDS,
        ZonesConstants.firstContainingOrDefault(
            null, ZonesConstants.Zone.class, ZonesConstants.Zone.OUT_OF_BOUNDS));
    assertEquals(
        ZonesConstants.Zone.OUT_OF_BOUNDS,
        ZonesConstants.firstContainingOrDefault(
            new Translation2d(1, 1), null, ZonesConstants.Zone.OUT_OF_BOUNDS));
  }

  @Test
  void firstContainingOrDefault_returnsProvidedDefaultZoneWhenNoMatch() {
    ZonesConstants.Zone providedDefault = ZonesConstants.Zone.BLUE_ZONE;

    ZonesConstants.Zone result =
        ZonesConstants.firstContainingOrDefault(
            new Translation2d(-999.0, -999.0), ZonesConstants.Zone.class, providedDefault);
    assertEquals(providedDefault, result);

    ZonesConstants.Zone nullEnumClassResult =
        ZonesConstants.firstContainingOrDefault(new Translation2d(1.0, 1.0), null, providedDefault);
    assertEquals(providedDefault, nullEnumClassResult);
  }

  @Test
  void bumpAngle_isFiveDegrees() {
    assertEquals(5.0, ZonesConstants.BUMP_ANGLE.in(Degrees), 1e-9);
  }

  @Test
  void enumName_returnsDeclaredIdentifier() {
    assertEquals("BLUE_ZONE", ZonesConstants.Zone.BLUE_ZONE.name());
    assertEquals("NEUTRAL_ZONE_RIGHT", ZonesConstants.Zone.NEUTRAL_ZONE_RIGHT.name());
    assertEquals("NEUTRAL_ZONE_LEFT", ZonesConstants.Zone.NEUTRAL_ZONE_LEFT.name());
    assertEquals("RED_ZONE", ZonesConstants.Zone.RED_ZONE.name());
    assertEquals("OUT_OF_BOUNDS", ZonesConstants.Zone.OUT_OF_BOUNDS.name());

    assertEquals("BLUE_OUTPOST_SIDE", ZonesConstants.Trench.BLUE_OUTPOST_SIDE.name());
    assertEquals("BLUE_DEPOT_SIDE", ZonesConstants.Trench.BLUE_DEPOT_SIDE.name());
    assertEquals("RED_OUTPOST_SIDE", ZonesConstants.Trench.RED_OUTPOST_SIDE.name());
    assertEquals("RED_DEPOT_SIDE", ZonesConstants.Trench.RED_DEPOT_SIDE.name());

    assertEquals("BLUE_HUB_DEADZONE", ZonesConstants.HubDeadZone.BLUE_HUB_DEADZONE.name());
    assertEquals("RED_HUB_DEADZONE", ZonesConstants.HubDeadZone.RED_HUB_DEADZONE.name());
  }
}
