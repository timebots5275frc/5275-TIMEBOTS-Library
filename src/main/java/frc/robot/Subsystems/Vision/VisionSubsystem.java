package frc.robot.Subsystems.Vision;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomTypes.Math.Vector3;

/**
 * Collects data from a single Limelight and exposes it through getters.
 * Contains no robot logic: other subsystems and commands read from here.
 */
public class VisionSubsystem extends SubsystemBase {

  private static final String LIMELIGHT_NAME = "limelight";

  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);


  // The gyro is used to get the robot's rotation on the field, which is combined with the Limelight's pose data to compute the robot's position on the field using Megatag2
  private Pigeon2 gyro;

  /** Latest MegaTag 2 estimate (WPILib blue-origin coordinates). May be null if the Limelight hasn't published one. */
  private LimelightHelpers.PoseEstimate mt2;

  /** Ignore MegaTag 2 estimates while the robot spins faster than this (deg/s), since gyro/camera latency makes them unreliable. */
  private static final double MAX_YAW_RATE_DEG_PER_SEC = 720.0;

  // Raw data
  private int aprilTagID = -1;
  private double horizontalOffset = 0.0; //degrees from crosshair to target; positive means target is to the right

  // Rolling buffers + averages
  private final ArrayList<Vector3> tagPosValues = new ArrayList<>();
  private final ArrayList<Vector3> tagRotValues = new ArrayList<>();
  private final ArrayList<Vector3> robotPosValues = new ArrayList<>();

  private Vector3 avgTagPos = Vector3.zero;
  private Vector3 avgTagRot = Vector3.zero;
  private Vector3 avgRobotPos = Vector3.zero;
  private Vector3 avgRobotRot = Vector3.zero;

  /** For use in triggers/commands, e.g. new Trigger(vision.hasValidData). */
  public final BooleanSupplier hasValidData = this::hasValidData;


  public VisionSubsystem(Pigeon2 gyro) {
    this.gyro = gyro;
  }

  //#region Periodic
  /**
   * Runs every robot loop. Reads the latest tag ID and horizontal offset from the
   * Limelight, updates the averaged pose data if a tag is visible (or clears the
   * buffers if not), and logs everything to SmartDashboard.
   */
  @Override
  public void periodic() {
    aprilTagID = (int) limelight.getEntry("tid").getDouble(-1.0);
    horizontalOffset = limelight.getEntry("tx").getDouble(0.0);

    // MegaTag 2 requires the robot's orientation to compute the robot's position on the field. The gyro yaw (and yaw rate)
    // are sent to the Limelight before reading the estimate. Pitch/roll are passed as 0 because they are not relevant for a flat field.
    LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, getGyroYawDegrees(), getGyroYawRateDegPerSec(), 0, 0, 0, 0);
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

    if (aprilTagID != -1) {
      updateTargetInRobotSpace();
      updateRobotInFieldSpace();
    } else {
      clearData();
    }

    logData();
  }

  // ---------------------------------------------------------------- Updating
  //#region Update Methods
  /**
   * Reads the tag's pose relative to the robot ({@code targetpose_robotspace}) and
   * updates the rolling average of its position and rotation. If the data looks
   * invalid, the previous averages are kept.
   */
  private void updateTargetInRobotSpace() {
    double[] vals = limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    if (vals.length < 6 || vals[0] == 0) {
      return; // bad data, keep previous averages
    }
    //x, y, z, roll, pitch, yaw)
    addToBuffer(tagPosValues, new Vector3(vals[0], vals[1], vals[2]));
    addToBuffer(tagRotValues, new Vector3(vals[3], vals[4], vals[5]));
    avgTagPos = average(tagPosValues);
    avgTagRot = average(tagRotValues);
  }

  /**
   * Updates the robot's field position from the MegaTag 2 estimate ({@link #mt2}) and
   * keeps a rolling average of the position. Rotation is taken directly from the gyro
   * (MegaTag 2 uses the gyro yaw as its heading) rather than averaged. The estimate is
   * rejected, and previous values are kept, if it is missing, has no tags, or the robot
   * is rotating too fast.
   */
  private void updateRobotInFieldSpace() {
    if (mt2 == null || mt2.tagCount == 0) {
      return;
    }
    if (Math.abs(getGyroYawRateDegPerSec()) > MAX_YAW_RATE_DEG_PER_SEC) {
      return;
    }

    addToBuffer(robotPosValues, new Vector3(mt2.pose.getX(), mt2.pose.getY(), 0));
    avgRobotPos = average(robotPosValues);
    avgRobotRot = new Vector3(0, 0, mt2.pose.getRotation().getDegrees());
  }

  /** @return the gyro's yaw in degrees (counter-clockwise positive). */
  private double getGyroYawDegrees() {
    return gyro.getYaw().getValueAsDouble();
  }

  /** @return the gyro's yaw rate about the world Z axis in degrees per second. */
  private double getGyroYawRateDegPerSec() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Empties all rolling buffers so stale readings don't pollute the average the
   * next time a tag is seen. The last computed averages are left as-is, but the
   * getters should only be trusted while {@link #hasValidData()} is true.
   */
  private void clearData() {
    tagPosValues.clear();
    tagRotValues.clear();
    robotPosValues.clear();
  }

  /**
   * Adds a new value to the front of a rolling buffer and drops the oldest value
   * if the buffer exceeds {@link VisionConstants#VALUES_TO_AVERAGE}.
   *
   * @param buffer the buffer to add to
   * @param newVal the newest reading
   */
  private static void addToBuffer(ArrayList<Vector3> buffer, Vector3 newVal) {
    buffer.add(0, newVal);
    if (buffer.size() > VisionConstants.VALUES_TO_AVERAGE) {
      buffer.remove(buffer.size() - 1);
    }
  }

  /**
   * Computes the component-wise average of every vector in a buffer.
   *
   * @param buffer the buffer to average
   * @return the average, or {@link Vector3#zero} if the buffer is empty
   */
  private static Vector3 average(ArrayList<Vector3> buffer) {
    if (buffer.isEmpty()) {
      return Vector3.zero;
    }

    Vector3 sum = Vector3.zero;
    for (Vector3 v : buffer) {
      sum = sum.add(v);
    }
    return sum.divideBy(buffer.size());
  }

  // ----------------------------------------------------------------- Logging
  //#region Logging
  /**
   * Publishes the current vision data to SmartDashboard under the {@code Vision/}
   * prefix. Shows "None" or "N/A" when no tag is visible.
   */
  private void logData() {
    boolean valid = hasValidData();
    SmartDashboard.putString("Vision/AprilTag ID", valid ? String.valueOf(aprilTagID) : "None");
    SmartDashboard.putString("Vision/Tag pos (robot space)", valid ? avgTagPos.toString(3) : "N/A");
    SmartDashboard.putString("Vision/Tag rot (robot space)", valid ? avgTagRot.toString(3) : "N/A");
    SmartDashboard.putString("Vision/Robot pos (field space)", valid ? avgRobotPos.toString(3) : "N/A");
    SmartDashboard.putString("Vision/Robot rot (field space)", valid ? avgRobotRot.toString(3) : "N/A");
    SmartDashboard.putNumber("Vision/Horizontal offset", horizontalOffset);
  }

  // ----------------------------------------------------------------- Control
  //#region Control
  /**
   * Forces the Limelight LEDs on or off, overriding the pipeline's LED setting.
   *
   * @param on true to force the LEDs on, false to force them off
   */
  public void setLEDs(boolean on) {
    limelight.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  // ----------------------------------------------------------------- Getters 
  //#region Getters

  /**
   * @return true if the Limelight currently sees an AprilTag
   */
  public boolean hasValidData() { return aprilTagID != -1; }

  /**
   * @return the ID of the primary detected AprilTag, or -1 if none is visible
   */
  public int getAprilTagID() { return aprilTagID; }

  /**
   * @return the horizontal angle in degrees from the crosshair to the target;
   *         positive means the target is to the right. 0 if no target is visible.
   */
  public double getHorizontalOffset() { return horizontalOffset; }

  /**
   * @return the averaged position of the tag relative to the robot (meters),
   *         or {@link Vector3#zero} if no data has been collected
   */
  public Vector3 getTagPosInRobotSpace() { return avgTagPos; }

  /**
   * @return the averaged rotation of the tag relative to the robot (degrees),
   *         or {@link Vector3#zero} if no data has been collected
   */
  public Vector3 getTagRotInRobotSpace() { return avgTagRot; }

  /**
   * @return the averaged MegaTag 2 position of the robot on the field (meters, x/y
   *         in the blue-alliance-origin coordinate system, z always 0), or
   *         {@link Vector3#zero} if no data has been collected
   */
  public Vector3 getRobotPosInFieldSpace() { return avgRobotPos; }

  /**
   * @return the robot's field rotation in degrees (roll, pitch, yaw) as used by
   *         MegaTag 2. Roll and pitch are always 0 and yaw comes from the gyro.
   *         {@link Vector3#zero} if no data has been collected.
   */
  public Vector3 getRobotRotInFieldSpace() { return avgRobotRot; }

  /**
   * @return the latest raw MegaTag 2 estimate (pose, timestamp, tag count, etc.),
   *         suitable for feeding a pose estimator. May be null.
   */
  public LimelightHelpers.PoseEstimate getMegaTag2Estimate() { return mt2; }
}