package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  public enum Field {
    Andymark,
    Welded
  }

  public static Field currentFieldType = Field.Welded;

  public static final Distance fieldLength =
      (currentFieldType == Field.Andymark)
          ? Feet.of(54.0).plus(Inches.of(2.12))
          : Feet.of(54.0).plus(Inches.of(3.2));

  public static final Distance fieldWidth =
      (currentFieldType == Field.Andymark)
          ? Feet.of(26.0).plus(Inches.of(4.64))
          : Feet.of(26.0).plus(Inches.of(5.7));

  public static final AprilTagFieldLayout fieldAprilTags =
      (currentFieldType == Field.Andymark)
          ? AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)
          : AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Pose2d blueHubPose =
      (currentFieldType == Field.Andymark)
          ? new Pose2d(
              Feet.of(15.0).plus(Inches.of(1.56)),
              Feet.of(13.0).plus(Inches.of(2.32)),
              Rotation2d.kZero)
          : new Pose2d(
              Feet.of(15.0).plus(Inches.of(2.11)),
              Feet.of(13.0).plus(Inches.of(2.85)),
              Rotation2d.kZero);

  public static Pose2d rotateFieldCentric(Pose2d pose, Rotation2d rotation) {
    return rotatePoseAroundPoint(
        pose, new Translation2d(fieldLength.div(2), fieldWidth.div(2)), rotation);
  }

  public static Pose2d rotatePoseAroundPoint(
      Pose2d pose, Translation2d point, Rotation2d rotation) {
    return new Pose2d(
        pose.getTranslation().rotateAround(point, rotation), pose.getRotation().plus(rotation));
  }

  public static Pose2d flipPoseForAlliance(Pose2d blueRelativePose, boolean redAlliance) {
    return redAlliance
        ? rotateFieldCentric(blueRelativePose, Rotation2d.k180deg)
        : blueRelativePose;
  }

  public static Rotation2d flipRotationForAlliance(
      Rotation2d blueRelativeRotation, boolean redAlliance) {
    return redAlliance ? blueRelativeRotation.plus(Rotation2d.k180deg) : blueRelativeRotation;
  }

  public static Pose2d getHubPoseForAlliance(boolean redAlliance) {
    return flipPoseForAlliance(blueHubPose, redAlliance);
  }
}
