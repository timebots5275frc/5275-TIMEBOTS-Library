package frc.robot.Subsystems.GlobalPose;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class TankGlobalRobotPoseEstimatorSubsystem extends GlobalRobotPoseSubsystem<DifferentialDriveWheelPositions, DifferentialDrivePoseEstimator> {

  private final DoubleSupplier leftMeters;
  private final DoubleSupplier rightMeters;

  /**
   * @param leftMeters  total left wheel distance traveled, in meters
   * @param rightMeters total right wheel distance traveled, in meters
   */
  public TankGlobalRobotPoseEstimatorSubsystem(
      VisionSubsystem vision,
      Pigeon2 gyro,
      DifferentialDriveKinematics kinematics,
      DoubleSupplier leftMeters,
      DoubleSupplier rightMeters) {
    super(
        vision,
        gyro,
        new DifferentialDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            leftMeters.getAsDouble(),
            rightMeters.getAsDouble(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(5)),    // odometry std devs (x, y, theta)
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30))));   // default vision std devs

    this.leftMeters = leftMeters;
    this.rightMeters = rightMeters;
  }

  @Override
  protected DifferentialDriveWheelPositions getWheelPositions() {
    return new DifferentialDriveWheelPositions(leftMeters.getAsDouble(), rightMeters.getAsDouble());
  }
}