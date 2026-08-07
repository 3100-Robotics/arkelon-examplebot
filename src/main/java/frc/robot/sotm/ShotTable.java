package frc.robot.sotm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.List;

/** Based on blog.eeshwark.com/blog/shooting-on-the-fly-pt2 */
public class ShotTable {
  public static final List<Pair<Distance, Double>> distanceAngleTable = new ArrayList<>();
  public static final List<Pair<Distance, Double>> distanceSpeedTable = new ArrayList<>();
  public static final List<Pair<Distance, Double>> distanceTofTable = new ArrayList<>();

  static {
    // Distance Angle
    distanceAngleTable.add(Pair.of(Meters.of(1.36), 20.0 - 2.5)); // Close
    distanceAngleTable.add(Pair.of(Inches.of(135), 27.8)); // Tower fwd
    distanceAngleTable.add(Pair.of(Inches.of(164), 31.5)); // Tower bkwd
    distanceAngleTable.add(Pair.of(Meters.of(5.86), 40.0 - 2.5)); // Far

    // Distance Speed
    distanceSpeedTable.add(Pair.of(Meters.of(1.36), 2400.0 + (2400 * 0.05))); // Close
    distanceSpeedTable.add(Pair.of(Inches.of(135), 3100.0)); // Tower fwd
    distanceSpeedTable.add(Pair.of(Inches.of(164), 3300.0)); // Tower bkwd
    distanceSpeedTable.add(Pair.of(Meters.of(5.86), 3960.0 + (3960 * 0.05))); // Far

    // Distance tof
    distanceTofTable.add(Pair.of(Meters.of(1.36), 0.40)); // Close
    distanceTofTable.add(Pair.of(Inches.of(135), 0.55)); // Tower fwd
    distanceTofTable.add(Pair.of(Inches.of(164), 0.62)); // Tower bkwd
    distanceTofTable.add(Pair.of(Meters.of(5.86), 0.95)); // Far
  }

  public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap SPEED_MAP = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();
  public static final double[] TUNED_DISTANCES_M;

  static {
    distanceAngleTable.forEach(p -> ANGLE_MAP.put(p.getFirst().in(Meters), p.getSecond()));
    distanceSpeedTable.forEach(p -> SPEED_MAP.put(p.getFirst().in(Meters), p.getSecond()));
    distanceTofTable.forEach(p -> TOF_MAP.put(p.getFirst().in(Meters), p.getSecond()));
    TUNED_DISTANCES_M =
        distanceTofTable.stream()
            .map(p -> p.getFirst().in(Meters))
            .sorted()
            .mapToDouble(d -> d)
            .toArray();
  }

  public static double horizontalVelocity(double distanceMeters) {
    return distanceMeters / TOF_MAP.get(distanceMeters);
  }

  public static double effectiveDistanceForVelocity(double requiredHorizVel) {
    double lastD = TUNED_DISTANCES_M[0], lastV = horizontalVelocity(lastD);
    if (requiredHorizVel <= lastV) return lastD;
    for (int i = 1; i < TUNED_DISTANCES_M.length; i++) {
      double d = TUNED_DISTANCES_M[i], v = horizontalVelocity(d);
      if (requiredHorizVel <= v) {
        return MathUtil.interpolate(lastD, d, (requiredHorizVel - lastV) / (v - lastV));
      }
      lastD = d;
      lastV = v;
    }
    return lastD;
  }
}
