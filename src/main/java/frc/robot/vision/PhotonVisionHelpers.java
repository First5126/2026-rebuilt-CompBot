package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Utility helpers for PhotonVision cameras and related measurements.
 *
 * <p>This class provides compatibility wrappers around different PhotonVision API versions,
 * instrumentation counters, and small convenience helpers such as distance calculations.
 */
public class PhotonVisionHelpers {

  private static final Map<Class<?>, CachedGetter> cameraGetterCache = new ConcurrentHashMap<>();
  // Instrumentation counters (runtime) to measure how often various code paths are used.
  private static final AtomicLong s_blockingInvokes = new AtomicLong(0);
  private static final AtomicLong s_syncInvokes = new AtomicLong(0);
  private static final AtomicLong s_legacyInvokes = new AtomicLong(0);

  private static class CachedGetter {
    enum Kind {
      LEGACY,
      BLOCKING,
      SYNC
    }

    final Kind kind;
    final Method method; // method to invoke for BLOCKING or SYNC

    CachedGetter(Kind kind, Method method) {
      this.kind = kind;
      this.method = method;
    }
  }

  /**
   * Retrieves the latest pipeline result from a Photon camera.
   *
   * @param camera photon camera instance
   * @return latest pipeline result
   */
  public static PhotonPipelineResult getResultOfCamera(PhotonCamera camera) {
    Class<?> clazz = camera.getClass();
    CachedGetter cg = cameraGetterCache.get(clazz);
    if (cg == null) {
      // resolve once per camera class
      CachedGetter.Kind kind = CachedGetter.Kind.LEGACY;
      Method method = null;
      try {
        method = clazz.getMethod("getLatestResultBlocking", long.class);
        kind = CachedGetter.Kind.BLOCKING;
      } catch (NoSuchMethodException e) {
        try {
          method = clazz.getMethod("getLatestResultSync");
          kind = CachedGetter.Kind.SYNC;
        } catch (NoSuchMethodException ex) {
          kind = CachedGetter.Kind.LEGACY;
          method = null;
        }
      }
      cg = new CachedGetter(kind, method);
      cameraGetterCache.put(clazz, cg);
    }

    try {
      switch (cg.kind) {
        case BLOCKING:
          Object res = cg.method.invoke(camera, 20L);
          s_blockingInvokes.incrementAndGet();
          if (res instanceof PhotonPipelineResult) {
            return (PhotonPipelineResult) res;
          }
          break;
        case SYNC:
          Object res2 = cg.method.invoke(camera);
          s_syncInvokes.incrementAndGet();
          if (res2 instanceof PhotonPipelineResult) {
            return (PhotonPipelineResult) res2;
          }
          break;
        case LEGACY:
        default:
          // fall through to legacy call below
          break;
      }
    } catch (ReflectiveOperationException e) {
      // If reflective invocation fails, print once and fall back.
      System.err.println("ERROR: PhotonCamera invocation failure: " + e.getMessage());
    }

    // Fallback to deprecated direct call
    PhotonPipelineResult result = camera.getLatestResult();
    s_legacyInvokes.incrementAndGet();
    return result;
  }

  /** Returns instrumentation counters for PhotonVisionHelpers. */
  public static String getInstrumentationReport() {
    return String.format(
        "PhotonVisionHelpers: resolveCacheSize=%d, blocking=%d, sync=%d, legacy=%d",
        cameraGetterCache.size(),
        s_blockingInvokes.get(),
        s_syncInvokes.get(),
        s_legacyInvokes.get());
  }

  /**
   * Computes the distance between two poses.
   *
   * @param pose1 first pose
   * @param pose2 second pose
   * @return distance in meters between the poses
   */
  public static double findDistance(Pose2d pose1, Pose2d pose2) {
    return PhotonUtils.getDistanceToPose(pose1, pose2);
  }

  /**
   * Computes the average distance from the robot pose to visible AprilTags.
   *
   * @param photonDetail configured PhotonVision camera details
   * @param robotPose2d current robot pose
   * @return average distance in meters, or 0.0 if no targets are visible
   */
  public static double getAverageDistanceBetweenTags(
      PhotonDetails photonDetail, Pose2d robotPose2d) {

    // Use centralized getter (which handles deprecated API suppression) so we only
    // have one place to update when PhotonVision removes the old API.
    PhotonPipelineResult pipelineResult = getResultOfCamera(photonDetail.camera);
    if (pipelineResult == null) {
      return 0.0;
    }

    List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
    if (targets == null || targets.isEmpty()) {
      return 0.0;
    }

    double totalDistanceOfTargets = 0.0;
    int numberOfValidTargets = 0;
    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> currentTagPose =
          AprilTagLocalizationConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId());
      if (currentTagPose.isPresent()) {
        numberOfValidTargets++;
        totalDistanceOfTargets +=
            PhotonUtils.getDistanceToPose(robotPose2d, currentTagPose.get().toPose2d());
      }
    }

    if (numberOfValidTargets == 0) {
      return 0.0;
    }

    return totalDistanceOfTargets / numberOfValidTargets;
  }
}
