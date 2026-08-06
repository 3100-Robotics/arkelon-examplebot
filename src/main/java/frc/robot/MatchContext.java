package frc.robot;

import com.sbdc.loggerhead.LightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

public class MatchContext extends LightSubsystem {

  public boolean isRedAlliance = false;
  public boolean allianceSet = false;

  public Pose2d getHubPose() {
    return FieldConstants.getHubPoseForAlliance(false);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !allianceSet) {
      DriverStation.getAlliance()
          .ifPresentOrElse(
              color -> {
                isRedAlliance = color == Alliance.Red;
                allianceSet = true;
              },
              () -> allianceSet = false);
    }
  }
}
