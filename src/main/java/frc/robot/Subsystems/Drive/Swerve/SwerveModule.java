package frc.robot.Subsystems.Drive.Swerve;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
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
 *   <li><b>Steering:</b> a single position PID on the SPARK MAX ({@code kPosition}), using
 *       the steer motor's built-in encoder converted to wheel degrees. The CANcoder is read
 *       once at startup (and on {@link #resyncSteerEncoder()}) to seed that encoder with the
 *       wheel's true angle. Position wrapping makes the SPARK take the shortest way around.</li>
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
    private SparkMax steerMotor;
    private CANcoder steerAngleEncoder; // absolute wheel angle, in rotations

    /** Short module name ("LF", "RF", "RR", "LR"), used as a prefix for dashboard keys. */
    private String name;

    // ------------------------------------------------------------------ Controllers

    /** Steering loop (SPARK MAX): holds a wheel angle in degrees using the built-in encoder. */
    private SparkClosedLoopController steerMotorPositionPID;

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
        steerMotorPositionPID = steer.getMotorController();

        configureSteerMotor();

        // Wait (briefly) for the CANcoder's first reading, then seed the steer motor's encoder with it.
        // Without the wait, a not-yet-published reading of 0 would start every wheel misaligned.
        steerAngleEncoder.getAbsolutePosition().waitForUpdate(0.25);
        resyncSteerEncoder();
    }

    /**
     * Applies the position-control settings on top of whatever the helper already configured
     * (current limits, etc. are left alone via kNoResetSafeParameters).
     */
    private void configureSteerMotor() { //TODO: make this into a helper
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(SwerveDriveConstants.STEER_MOTOR_INVERTED);

        // Make the built-in encoder report WHEEL DEGREES instead of motor rotations
        config.encoder.positionConversionFactor(360.0 / SwerveDriveConstants.STEER_GEAR_RATIO);

        // Feedback is the primary (built-in) encoder, which is the default for a brushless motor
        config.closedLoop
            .pid(SwerveDriveConstants.STEER_PID.p,SwerveDriveConstants.STEER_PID.i, SwerveDriveConstants.STEER_PID.d)// is this necsascary if already set through the helpers
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)          // take the shortest path around the circle
            .positionWrappingInputRange(-180, 180); // wheel angle range, in degrees

        steerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Copies the CANcoder's absolute angle into the steer motor's built-in encoder. Runs at
     * startup. It can also be called while the robot is disabled and sitting still to remove
     * any drift between the motor encoder and the true wheel angle.
     */
    public void resyncSteerEncoder() {
        steerMotor.getEncoder().setPosition(steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    }

    // ------------------------------------------------------------------ State

    /**
     * Returns the wheel's steering angle from the steer motor's built-in encoder, wrapped to
     * -180..180 degrees (the encoder itself keeps counting past +/-180).
     */
    private Rotation2d getSteerAngle() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(steerMotor.getEncoder().getPosition(), -180, 180));
    }

    /**
     * Returns the current state of the module.
     *
     * @return the current wheel speed (m/s) and steering angle
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveMotor.getEncoder().getVelocity());

        return new SwerveModuleState(driveSpeed, getSteerAngle());
    }

    /**
     * Returns how far the wheel has driven and its current steering angle. Used by odometry.
     *
     * <p>The distance is negated to match {@link #speedFromDriveRpm(double)} (the drive
     * direction is flipped by the gearing), so driving backward reduces the distance.
     *
     * @return the module's position (signed meters driven + steering angle)
     */
    public SwerveModulePosition getPosition() {
        double distance = -driveMotor.getEncoder().getPosition() * SwerveDriveConstants.DRIVE_GEAR_RATIO * SwerveDriveConstants.WHEEL_CIRCUMFERENCE;

        return new SwerveModulePosition(distance, getSteerAngle());
    }

    // ------------------------------------------------------------------ Control

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState desired wheel speed (m/s) and steering angle
     * @param logValues    true to publish debugging values to SmartDashboard
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logValues) {

        Rotation2d currentAngle = getSteerAngle();

        // Copy first, because optimize() modifies the object it's called on
        // (callers such as alignWheels() reuse one state object across modules)
        SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

        // Optimize the state to avoid rotating the wheel more than 90 degrees
        // (it may flip the angle by 180 and reverse the wheel speed instead)
        state.optimize(currentAngle);

        // Steering: command the wheel angle (degrees) directly. The SPARK runs the position PID.
        steerMotorPositionPID.setSetpoint(state.angle.getDegrees(), SparkMax.ControlType.kPosition);

        // Driving: convert wheel speed (m/s) to motor RPM
        double driveMotorRpm = driveRpmFromSpeed(state.speedMetersPerSecond);

        if (logValues) {
            double driveSpeed = driveMotor.getEncoder().getVelocity();
            SmartDashboard.putNumber(name + " DriveSpeedMetersPerSecond", state.speedMetersPerSecond);
            SmartDashboard.putNumber(name + " DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber(name + " DriveMotorSpeed", driveSpeed);
            SmartDashboard.putNumber(name + " SteerTargetDeg", state.angle.getDegrees());
            SmartDashboard.putNumber(name + " SteerActualDeg", currentAngle.getDegrees());
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

    //#region Getters
    public SparkFlex getDriveMotor() {
        return driveMotor;
    }
    public SparkMax getSteerMotor() {
        return steerMotor;
    }
}