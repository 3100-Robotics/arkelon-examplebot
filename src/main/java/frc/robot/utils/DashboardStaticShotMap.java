package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardStaticShotMap implements ShotMap {
  public DashboardStaticShotMap() {
    SmartDashboard.putNumber("Hood Angle", 31);
    SmartDashboard.putNumber("Flywheel Speed", 3100);
  }

  public String getTarget() {
    return "Hub";
  }

  public AngularVelocity getFlywheelSpeed() {
    return RPM.of(SmartDashboard.getNumber("Flywheel Speed", 5100));
  }

  public Angle getHoodAngle() {
    return Degrees.of(SmartDashboard.getNumber("Hood Angle", 51));
  }
}
