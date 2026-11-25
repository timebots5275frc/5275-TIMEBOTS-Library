package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * One vision-derived robot pose for the pose estimator.
 *
 * @param pose             field-relative robot pose (blue-origin)
 * @param timestampSeconds FPGA time at which the image was captured
 * @param stdDevs          trust in this measurement as (x meters, y meters, theta radians); smaller = more trust
 */
public record VisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}