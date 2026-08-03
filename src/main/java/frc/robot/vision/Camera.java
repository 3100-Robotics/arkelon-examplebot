package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class Camera {
  public final AprilTagFieldLayout tagLayout;
  public final Transform3d robotToCam;
  public final String camName;
  public final PhotonCamera camera;
  public final PhotonPoseEstimator photonEstimator;

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 1);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1, 1, 1);

  private Matrix<N3, N1> curStdDevs = VecBuilder.fill(0, 0, 0);

  public SimCameraProperties simCamProps;
  public PhotonCameraSim sim;

  public Camera(
      AprilTagFieldLayout atagfieldlayout,
      Transform3d robotCameraTransform,
      String camName,
      boolean doSim,
      SimCameraProperties simCamProps,
      boolean simDrawWireframe) {
    tagLayout = atagfieldlayout;
    robotToCam = robotCameraTransform;
    this.camName = camName;
    camera = new PhotonCamera(this.camName);
    photonEstimator = new PhotonPoseEstimator(this.tagLayout, this.robotToCam);

    if (doSim) {
      this.simCamProps = simCamProps;
      this.sim = new PhotonCameraSim(this.camera, this.simCamProps);
      this.sim.enableDrawWireframe(simDrawWireframe);
    }
  }
}
