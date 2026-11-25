package frc.robot.Subsystems.GlobalPose;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.VisionMeasurement;
import frc.robot.Subsystems.Vision.VisionSubsystem;

/**
 * @param <W> wheel positions type (e.g. DifferentialDriveWheelPositions)
 * @param <E> the matching PoseEstimator (e.g. DifferentialDrivePoseEstimator)
 */
public abstract class GlobalRobotPoseSubsystem<W, E extends PoseEstimator<W>> extends SubsystemBase {

  protected final VisionSubsystem vision;
  protected final Pigeon2 gyro;
  protected final E poseEstimator;

  private final Field2d field = new Field2d();

  protected GlobalRobotPoseSubsystem(VisionSubsystem vision, Pigeon2 gyro, E poseEstimator) {
    this.vision = vision;
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;
    SmartDashboard.putData("GlobalPose/Field", field);
  }

  /** Current wheel distances (meters) in the estimator's native type. */
  protected abstract W getWheelPositions();

  @Override
  public void periodic() {
    // 1. Odometry
    poseEstimator.update(gyro.getRotation2d(), getWheelPositions());

    // 2. Vision
    
    List<VisionMeasurement> measurements = vision.getMeasurements();
    for (VisionMeasurement m : measurements) {
      poseEstimator.addVisionMeasurement(m.pose(), m.timestampSeconds(), m.stdDevs());
    }

    // 3. Telemetry
    field.setRobotPose(getPose());
  }

  //* Returns the Pose */ 
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Re-zeroes the estimator to the given pose. Wire this to PathPlanner/Choreo's resetPose. */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getWheelPositions(), pose);
  }
}