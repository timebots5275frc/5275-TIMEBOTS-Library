package frc.robot.Subsystems.Drive.Swerve;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.List;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

import frc.robot.CustomTypes.Math.Vector2;

/**
 * Four-module swerve drivetrain. Turns joystick-style speeds into wheel speeds and
 * angles for each {@link SwerveModule}, and tracks the robot's position with odometry
 * (wheel distances + Pigeon 2 gyro heading).
 *
 * <p><b>Module order:</b> anything that is an array of modules (kinematics, module
 * states, module positions) uses the same order everywhere:
 * left front (LF), right front (RF), right rear (RR), left rear (LR).
 */
public class SwerveDriveSubsystem extends SubsystemBase {

  //#region Hardware and kinematics (field initialization order matters, don't reorder)

  // Wheel positions relative to the robot center (meters, +X forward, +Y left)
  private final Translation2d leftFrontWheelLocation  = new Translation2d(SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_FRONT_WHEEL_X,  SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_FRONT_WHEEL_Y);
  private final Translation2d rightFrontWheelLocation = new Translation2d(SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_FRONT_WHEEL_X, SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_FRONT_WHEEL_Y);
  private final Translation2d rightRearWheelLocation  = new Translation2d(SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_REAR_WHEEL_X,  SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_REAR_WHEEL_Y);
  private final Translation2d leftRearWheelLocation   = new Translation2d(SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_REAR_WHEEL_X,   SwerveDriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_REAR_WHEEL_Y);

  // The four modules (drive motor data, steer motor data, steer encoder CAN ID, name)
  private final SwerveModule leftFrontSwerveModule  = new SwerveModule(SwerveDriveConstants.LEFT_FRONT_DRIVE_MOTOR_DATA,  SwerveDriveConstants.LEFT_FRONT_STEER_MOTOR_DATA,  SwerveDriveConstants.LEFT_FRONT_STEER_ENCODER_ID,  "LF");
  private final SwerveModule rightFrontSwerveModule = new SwerveModule(SwerveDriveConstants.RIGHT_FRONT_DRIVE_MOTOR_DATA, SwerveDriveConstants.RIGHT_FRONT_STEER_MOTOR_DATA, SwerveDriveConstants.RIGHT_FRONT_STEER_ENCODER_ID, "RF");
  private final SwerveModule rightRearSwerveModule  = new SwerveModule(SwerveDriveConstants.RIGHT_REAR_DRIVE_MOTOR_DATA,  SwerveDriveConstants.RIGHT_REAR_STEER_MOTOR_DATA,  SwerveDriveConstants.RIGHT_REAR_STEER_ENCODER_ID,  "RR");
  private final SwerveModule leftRearSwerveModule   = new SwerveModule(SwerveDriveConstants.LEFT_REAR_DRIVE_MOTOR_DATA,   SwerveDriveConstants.LEFT_REAR_STEER_MOTOR_DATA,   SwerveDriveConstants.LEFT_REAR_STEER_ENCODER_ID,   "LR");

  // Gyro: provides the robot heading (yaw)
  Pigeon2 pigeon2Gyro;

  // Converts between chassis motion and per-wheel states
  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontWheelLocation, rightFrontWheelLocation, rightRearWheelLocation, leftRearWheelLocation);

  // Module positions captured at startup
  public final SwerveModulePosition[] startingSwerveModulePositions = new SwerveModulePosition[] {leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(), rightRearSwerveModule.getPosition(), leftRearSwerveModule.getPosition()};

  // Tracks the robot's field position from the gyro heading and wheel distances
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, this.getHeading(), startingSwerveModulePositions);

  // Most recent module positions fed to odometry (refreshed every loop in updateOdometry())
  SwerveModulePosition[] currentSwerveModulePositions = startingSwerveModulePositions;

  /** When true, field-relative driving is disabled and the robot drives robot-relative instead. */
  private boolean flip;
  //#endregion

  /** Creates the drivetrain. Field-relative driving is enabled by default. */
  public SwerveDriveSubsystem(Pigeon2 pigeon2Gyro) {
    this.pigeon2Gyro = pigeon2Gyro;
    flip = false;
    
    //allow for live pid tuning for drive and steer velocity pids
    SwerveDriveConstants.DRIVE_PID.tunable("Drive PID",leftFrontSwerveModule.getDriveMotor(),leftRearSwerveModule.getDriveMotor(),rightFrontSwerveModule.getDriveMotor(),rightRearSwerveModule.getDriveMotor());
    SwerveDriveConstants.STEER_PID.tunable("Steer Velocity PID", leftFrontSwerveModule.getSteerMotor(),leftRearSwerveModule.getSteerMotor(),rightFrontSwerveModule.getSteerMotor(),rightRearSwerveModule.getSteerMotor());
  }

  /** Runs every loop: updates odometry and publishes the heading, positions, and pose. */
  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putString("positions", currentSwerveModulePositions[0].distanceMeters + " " + currentSwerveModulePositions[1].distanceMeters + " " + currentSwerveModulePositions[2].distanceMeters + " " + currentSwerveModulePositions[3].distanceMeters + " ");
    SmartDashboard.putNumber("left front pos", currentSwerveModulePositions[0].distanceMeters);
    SmartDashboard.putString("Odometry Pos", getOdometryPosition().toString());
    SmartDashboard.putString("Odometry Rot", m_odometry.getPoseMeters().getRotation().toString());
    

    //Live PID tuning for drive and steer velocity pids
    SwerveDriveConstants.DRIVE_PID.updateLive();
    SwerveDriveConstants.STEER_PID.updateLive();
  }

  //#region Driving

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward), m/s.
   * @param ySpeed        Speed of the robot in the y direction (sideways, left positive), m/s.
   * @param rot           Angular rate of the robot (counter-clockwise positive), rad/s.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Field-relative driving can be overridden by the flip toggle (see flipFieldRelative())
    boolean useFieldRelative = fieldRelative && !flip;

    // Field-relative: "forward" is away from the driver station regardless of robot heading.
    // Robot-relative: "forward" is the direction the robot faces.
    ChassisSpeeds speeds = useFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getHeading())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    // Convert chassis motion to a speed + angle for each wheel
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    // Index 3 is the left rear module
    SmartDashboard.putNumber("LRDesiredM/S", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("LRRotDeg", swerveModuleStates[3].angle.getDegrees());

    // If any wheel would exceed the max speed, scale all of them down together
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.MAX_DRIVE_SPEED);
    SmartDashboard.putBoolean("Field Relative", useFieldRelative);

    leftFrontSwerveModule.setDesiredState(swerveModuleStates[0], true);
    rightFrontSwerveModule.setDesiredState(swerveModuleStates[1], true);
    rightRearSwerveModule.setDesiredState(swerveModuleStates[2], true);
    leftRearSwerveModule.setDesiredState(swerveModuleStates[3], true);
  }

  /** Toggles forcing robot-relative driving (useful if the gyro is misbehaving). */
  public void flipFieldRelative() { flip = !flip; }

  /**
   * Sets the swerve ModuleStates directly (e.g. from a path-following command).
   * Order must be LF, RF, RR, LR.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.MAX_DRIVE_SPEED);

    leftFrontSwerveModule.setDesiredState(desiredStates[0], true);
    rightFrontSwerveModule.setDesiredState(desiredStates[1], true);
    rightRearSwerveModule.setDesiredState(desiredStates[2], true);
    leftRearSwerveModule.setDesiredState(desiredStates[3], true);
  }

  /** Points every wheel straight ahead at zero speed (useful for calibrating the encoder zeros). */
  public void alignWheels() {
    SwerveModuleState desiredStates = new SwerveModuleState(0, new Rotation2d(0));

    leftFrontSwerveModule.setDesiredState(desiredStates, true);
    rightFrontSwerveModule.setDesiredState(desiredStates, true);
    rightRearSwerveModule.setDesiredState(desiredStates, true);
    leftRearSwerveModule.setDesiredState(desiredStates, true);
  }
  //#endregion

  //#region Odometry

  /**
   * Reads the current position of every module, in the order LF, RF, RR, LR.
   *
   * @return a new array with each module's driven distance and steering angle
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        leftFrontSwerveModule.getPosition(),
        rightFrontSwerveModule.getPosition(),
        rightRearSwerveModule.getPosition(),
        leftRearSwerveModule.getPosition()
    };
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** @return the odometry X/Y position (meters) as a {@link Vector2}, without rotation. */
  public Vector2 getOdometryPosition()
  {
    Pose2d pose = m_odometry.getPoseMeters();
    return new Vector2(pose.getX(), pose.getY());
  }

  /**
   * Updates the field relative position of the robot using the gyro heading and a fresh
   * reading of every module. Called every loop from {@link #periodic()}, so odometry stays
   * current no matter how the modules are being driven (drive(), setModuleStates(), etc.).
   */
  public void updateOdometry() {
    currentSwerveModulePositions = getModulePositions();
    m_odometry.update(this.getHeading(), currentSwerveModulePositions);
  }

  /**
   * Resets the odometry Position and Angle to 0.
   */
  public void resetOdometry() {
    System.out.println("resetOdometry");
    m_odometry.resetPosition(this.getHeading(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Resets the odometry to the specified pose, using the current gyro heading and module positions.
   *
   * <p>Note: this does not change the gyro itself. If the pose's rotation should also be what the
   * gyro reports (e.g. so field-relative driving matches), call {@link #setGyroYaw(double)} too.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometryWithPose2d(Pose2d pose) {
    System.out.println("resetOdometryWithPose2d");
    m_odometry.resetPosition(this.getHeading(), getModulePositions(), pose);
  }
  //#endregion

  //#region Gyro

  /** Zeroes the gyro yaw. Note: this does not reset odometry. */
  public void resetPigeon() {
    pigeon2Gyro.setYaw(0);
  }

  /** @param degrees the yaw value (degrees) the gyro should now report */
  public void setGyroYaw(double degrees) {
    pigeon2Gyro.setYaw(degrees);
  }

  /**
   * Returns the heading of the robot from the Pigeon 2 (counter-clockwise positive).
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(pigeon2Gyro.getYaw().getValueAsDouble());
  }

  /** @return the raw gyro yaw in degrees */
  public double getGyroYawInDegrees() { return pigeon2Gyro.getYaw().getValueAsDouble(); }
  //#endregion

  //#region Trajectories

  /**
   * Generates a trajectory through the given poses.
   *
   * @param config trajectory constraints (max speed, acceleration, etc.)
   * @param list   the poses the trajectory should pass through
   * @return the generated trajectory
   */
  public static Trajectory generateTrajectory(TrajectoryConfig config, List<Pose2d> list) {
    return TrajectoryGenerator.generateTrajectory(list, config);
  }
  //#endregion
}