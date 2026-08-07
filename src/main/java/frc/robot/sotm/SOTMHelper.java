package frc.robot.sotm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.MatchContext;

/** Based on blog.eeshwark.com/blog/shooting-on-the-fly-pt2 */
public class SOTMHelper {
  public static double latencySeconds = 0.1;

  public record Solution(Rotation2d aimHeading, double hoodAngleDeg, double flywheelRpm) {}

  public static Solution solve(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());
    Translation2d robotVel =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Translation2d futurePos = robotPose.getTranslation().plus(robotVel.times(latencySeconds));

    Translation2d toGoal = MatchContext.getInstance().getHubTranslation().minus(futurePos);
    double distance = toGoal.getNorm();

    double baselineHorizVel = ShotTable.horizontalVelocity(distance);
    Translation2d targetVel = toGoal.div(distance).times(baselineHorizVel);

    Translation2d shotVel = targetVel.minus(robotVel);

    double effectiveDistance = ShotTable.effectiveDistanceForVelocity(shotVel.getNorm());

    return new Solution(
        shotVel.getAngle(),
        MathUtil.clamp(ShotTable.ANGLE_MAP.get(effectiveDistance), 12.667292, 40.0),
        ShotTable.SPEED_MAP.get(effectiveDistance));
  }
}
