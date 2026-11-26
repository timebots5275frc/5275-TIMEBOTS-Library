package frc.robot.Subsystems.Drive.Swerve;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.CustomTypes.Helpers;
import frc.robot.CustomTypes.MotorData;

/**
 * One swerve module: a drive motor (SPARK Flex), a steer motor (SPARK MAX), and a
 * CANcoder that measures the wheel's absolute steering angle.
 *
 * <p>Control structure:
 * <ul>
 *   <li><b>Steering:</b> cascaded control. An angle PID on the roboRIO
 *       ({@link #steerAnglePID}) compares the CANcoder angle to the target angle and
 *       outputs a steer motor RPM. A velocity PID on the SPARK MAX
 *       ({@link #steerMotorVelocityPID}) then holds that RPM.</li>
 *   <li><b>Driving:</b> a single velocity PID on the SPARK Flex
 *       ({@link #driveMotorVelocityPID}), fed with the motor RPM that corresponds to
 *       the desired wheel speed.</li>
 * </ul>
 *
 * <p>The CANcoder zero offset is not applied in code; it must be set on the
 * CANcoder itself (e.g. in Phoenix Tuner X) with the wheel pointing straight forward.
 */
public class SwerveModule {

    // ------------------------------------------------------------------ Hardware

    public SparkFlex driveMotor;
    @SuppressWarnings("unused")
    private SparkMax steerMotor;
    private CANcoder steerAngleEncoder; // absolute wheel angle, in rotations

    /** Short module name ("LF", "RF", "RR", "LR"), used as a prefix for dashboard keys. */
    private String name;

    // ------------------------------------------------------------------ Controllers

    /** Outer steering loop (roboRIO): angle error in degrees -> steer motor RPM. */
    private PIDController steerAnglePID;

    /** Inner steering loop (SPARK MAX): holds the steer motor RPM requested by the angle PID. */
    private SparkClosedLoopController steerMotorVelocityPID;

    /** Drive loop (SPARK Flex): holds the drive motor RPM. */
    private SparkClosedLoopController driveMotorVelocityPID;

    // ------------------------------------------------------------------ Setup

    /**
     * @param driveMotorData motor configuration (ID, limits, PID) for the drive motor
     * @param steerMotorData motor configuration (ID, limits, PID) for the steer motor
     * @param steerEncoderId CAN ID of the steering CANcoder
     * @param n              short name of this module, e.g. "LF"
     */
    public SwerveModule(MotorData driveMotorData, MotorData steerMotorData, int steerEncoderId, String n) {
        this.name = n;

        // Absolute steering angle encoder
        steerAngleEncoder = new CANcoder(steerEncoderId);

        // Drive motor + its onboard closed-loop controller
        var drive = Helpers.InstantiateFlex(driveMotorData);
        driveMotor = drive.getMotor();
        driveMotorVelocityPID = drive.getMotorController();

        // Steer motor + its onboard closed-loop controller
        var steer = Helpers.InstantiateMax(steerMotorData);
        steerMotor = steer.getMotor();
        steerMotorVelocityPID = steer.getMotorController();

        // Outer steering angle PID. Continuous input lets it take the short way around
        // the circle (e.g. 179 deg -> -179 deg is a 2 degree move, not 358).
        steerAnglePID = new PIDController(
            SwerveDriveConstants.STEER_ANGLE_PID.p,
            SwerveDriveConstants.STEER_ANGLE_PID.i,
            SwerveDriveConstants.STEER_ANGLE_PID.d);
        steerAnglePID.enableContinuousInput(-180, 180);

        //TODO: live pid tuning
    }

    // ------------------------------------------------------------------ State

    /**
     * Returns the current state of the module.
     *
     * @return the current wheel speed (m/s) and steering angle
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveMotor.getEncoder().getVelocity());
        double steerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360);

        return new SwerveModuleState(driveSpeed, new Rotation2d(steerAngleRadians));
    }

    /**
     * Returns how far the wheel has driven and its current steering angle. Used by odometry.
     *
     * <p>The distance is negated to match {@link #speedFromDriveRpm(double)} (the drive
     * direction is flipped by the gearing), so driving backward reduces the distance.
     * The angle comes straight from the CANcoder, which reports rotations.
     *
     * @return the module's position (signed meters driven + steering angle)
     */
    public SwerveModulePosition getPosition() {
        double distance = -driveMotor.getEncoder().getPosition() * SwerveDriveConstants.DRIVE_GEAR_RATIO * SwerveDriveConstants.WHEEL_CIRCUMFERENCE;
        Rotation2d angle = Rotation2d.fromRotations(steerAngleEncoder.getAbsolutePosition().getValueAsDouble());

        return new SwerveModulePosition(distance, angle);
    }

    // ------------------------------------------------------------------ Control

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState desired wheel speed (m/s) and steering angle
     * @param logValues    true to publish drive debugging values to SmartDashboard
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logValues) {

        // Current wheel angle from the absolute encoder (rotations -> degrees)
        double steerAngleDegrees = steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        Rotation2d currentAngle = Rotation2d.fromDegrees(steerAngleDegrees);

        // Copy first, because optimize() modifies the object it's called on
        // (callers such as alignWheels() reuse one state object across modules)
        SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

        // Optimize the state to avoid rotating the wheel more than 90 degrees
        // (it may flip the angle by 180 and reverse the wheel speed instead)
        state.optimize(currentAngle);

        // Steering: the output of steerAnglePID becomes the steer motor RPM reference.
        // The sign is flipped to match the steer motor's direction.
        double steerMotorRpm = steerAnglePID.calculate(steerAngleDegrees, state.angle.getDegrees());
        steerMotorVelocityPID.setSetpoint(-steerMotorRpm, SparkMax.ControlType.kVelocity);

        // Driving: convert wheel speed (m/s) to motor RPM
        double driveMotorRpm = driveRpmFromSpeed(state.speedMetersPerSecond);

        if (logValues) {
            double driveSpeed = driveMotor.getEncoder().getVelocity();
            SmartDashboard.putNumber(name + " DriveSpeedMetersPerSecond", state.speedMetersPerSecond);
            SmartDashboard.putNumber(name + " DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber(name + " DriveMotorSpeed", driveSpeed);
        }

        driveMotorVelocityPID.setSetpoint(driveMotorRpm, SparkMax.ControlType.kVelocity);
    }

    // ------------------------------------------------------------------ Conversions

    /**
     * Returns the required motor RPM from the desired wheel speed in meters/second.
     *
     * @param speedMetersPerSecond desired wheel speed
     * @return RPM of the drive motor (negated to account for the direction flip from the gearing)
     */
    public double driveRpmFromSpeed(double speedMetersPerSecond) {
        double rpm = speedMetersPerSecond * 60.0 / SwerveDriveConstants.WHEEL_CIRCUMFERENCE / SwerveDriveConstants.DRIVE_GEAR_RATIO;
        return -rpm; // rotation flip from gear count
    }

    /**
     * Returns the wheel speed in meters/second calculated from the drive motor RPM.
     *
     * @param rpm drive motor RPM
     * @return wheel speed in m/s (negated to account for the direction flip from the gearing)
     */
    public double speedFromDriveRpm(double rpm) {
        double speedMetersPerSecond = rpm * SwerveDriveConstants.DRIVE_GEAR_RATIO * SwerveDriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
        return -1 * speedMetersPerSecond; // Rotation reversed due to gears.
    }
}