package frc.robot.Subsystems.Drive.Swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TankDriveConstants;
import frc.robot.CustomTypes.Helpers;
import frc.robot.CustomTypes.MotorData;

public class SwerveModule {

    public SparkFlex driveMotor;
    private SparkMax steerMotor;
    private CANcoder steerAngleEncoder;

    private String name;

    private PIDController steerAnglePID;
    private SparkClosedLoopController steerMotorVelocityPID;
    private SparkClosedLoopController driveMotorVelocityPID;



    public SwerveModule(MotorData driveMotorData, MotorData steerMotorData, int steerEncoderId, String n) {
        // instantiate drive motor+controller, steer motor+velocity controller+anglePIDController, and steer angle encoder
        this.name = n; // set name

        steerAngleEncoder = new CANcoder(steerEncoderId); // instantiate steer angle encoder

        var drive = Helpers.InstantiateFlex(driveMotorData); // instantiate drive motor+controller
        driveMotor = drive.getMotor();
        driveMotorVelocityPID = drive.getMotorController();

        var steer = Helpers.InstantiateMax(steerMotorData); // instantiate steer motor+controller
        steerMotor = steer.getMotor();
        steerMotorVelocityPID = steer.getMotorController();


        /// Steer Angle PID ///
        steerAnglePID = new PIDController(SwerveDriveConstants.STEER_ANGLE_PID.p, SwerveDriveConstants.STEER_ANGLE_PID.i, SwerveDriveConstants.STEER_ANGLE_PID.d);
        steerAnglePID.enableContinuousInput(-180, 180);

        //TODO: live pid tuning
       
    }

    /**
     * Returns the current state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveMotor.getEncoder().getVelocity());
        double steerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360);

        return new SwerveModuleState(driveSpeed, new Rotation2d(steerAngleRadians));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logValues) {

        double steerAngleDegrees = steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        Rotation2d currentAngle = Rotation2d.fromDegrees(steerAngleDegrees);

        // Copy first, because optimize() modifies the object it's called on
        SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

        // Optimize the state to avoid rotating the wheel more than 90 degrees
        state.optimize(currentAngle);

        // The output of the steerAnglePID becomes the steer motor rpm reference.
        double steerMotorRpm = steerAnglePID.calculate(steerAngleDegrees, state.angle.getDegrees());

        steerMotorVelocityPID.setSetpoint(-steerMotorRpm, SparkMax.ControlType.kVelocity);

        double driveMotorRpm = driveRpmFromSpeed(state.speedMetersPerSecond);

        if (logValues) {
            double driveSpeed = driveMotor.getEncoder().getVelocity();
            SmartDashboard.putNumber(name + " DriveSpeedMetersPerSecond", state.speedMetersPerSecond);
            SmartDashboard.putNumber(name + " DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber(name + " DriveMotorSpeed", driveSpeed);
        }

        driveMotorVelocityPID.setSetpoint(driveMotorRpm, SparkMax.ControlType.kVelocity);
    }

    /**
     * Returns the required motor rpm from the desired wheel speed in meters/second
     * 
     * @param speedMetersPerSecond
     * @return rpm of the motor
     */
    public double driveRpmFromSpeed(double speedMetersPerSecond) {
        double rpm = speedMetersPerSecond * 60.0 / SwerveDriveConstants.WHEEL_CIRCUMFERENCE / SwerveDriveConstants.DRIVE_GEAR_RATIO;
        return -rpm; // rotation flip from gear count
    }

    /**
     * Returns the wheel speed in meters/second calculated from the drive motor rpm.
     * 
     * @param rpm
     * @return wheelSpeed
     */
    public double speedFromDriveRpm(double rpm) {
        double speedMetersPerSecond = rpm * SwerveDriveConstants.DRIVE_GEAR_RATIO * SwerveDriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
        return -1 * speedMetersPerSecond; // Rotation reversed due to gears.
    }

    public SwerveModulePosition getPosition() {
        double distance = Math.abs(driveMotor.getEncoder().getPosition() * SwerveDriveConstants.DRIVE_GEAR_RATIO * SwerveDriveConstants.WHEEL_CIRCUMFERENCE);
        // if (name.equals("LR")||name.equals("RR")){
        //     distance *=-1;
        // }
        return new SwerveModulePosition(distance, new Rotation2d(Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValueAsDouble())));
    }
}