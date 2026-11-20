package frc.robot.Subsystems.Drive;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.CustomTypes.Helpers;
import frc.robot.CustomTypes.PID;
import frc.robot.CustomTypes.Helpers.MotorAndController;

public class TankDriveSubsystem extends SubsystemBase {

    private SparkMax leftLeaderMotor;
    private SparkMax rightLeaderMotor;

    private SparkMax leftFollowerMotor;
    private SparkMax rightFollowerMotor;

    private SparkClosedLoopController leftMotorController;
    private SparkClosedLoopController rightMotorController;

    

  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem() {
    initMotors();
  }

  private void initMotors() {

    //set limits
    DriveConstants.DRIVE_PID.setFreeLimit(DriveConstants.DRIVE_FREE_LIMIT);
    DriveConstants.DRIVE_PID.setStallLimit(DriveConstants.DRIVE_STALL_LIMIT);
    
    //left leader
    MotorAndController<SparkMax> leftLeader = Helpers.InstantiateMotor(leftLeaderMotor, leftMotorController, DriveConstants.LEFT_LEADER_DATA);
    leftLeaderMotor = leftLeader.motor();
    leftMotorController = leftLeader.motorController();

    //right leader
    MotorAndController<SparkMax> rightLeader = Helpers.InstantiateMotor(rightLeaderMotor, rightMotorController, DriveConstants.LEFT_LEADER_DATA);
    rightLeaderMotor = rightLeader.motor();
    rightMotorController = rightLeader.motorController();

    //TODO instantiate followers; make sure to be following

    

    //Followers
    leftFollowerMotor = new SparkMax(Constants.DriveConstants.LEFT_FOLLOWER_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config1 = Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(leftLeaderMotor, IdleMode.kCoast);
    config1.follow(leftLeaderMotor, false); 
    leftFollowerMotor.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightFollowerMotor = new SparkMax(Constants.DriveConstants.RIGHT_FOLLOWER_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config2 = Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(rightLeaderMotor, IdleMode.kCoast);
    config2.follow(rightLeaderMotor, false); 
    rightFollowerMotor.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   *  
   * @param xSpeed 
   * meters per second
   * @param zRotation
   * degrees per second, positive is Counter Clockwise
   */
    public void arcadeDrive(double xSpeed, double zRotation) {


      zRotation *= -1;


      xSpeed = Math.min(xSpeed, Constants.DriveConstants.MAX_X_SPEED);
      xSpeed = Math.max(xSpeed, -Constants.DriveConstants.MAX_X_SPEED);

      zRotation = Math.min(zRotation, Constants.DriveConstants.MAX_Z_ROTATION);
      zRotation = Math.max(zRotation, -Constants.DriveConstants.MAX_Z_ROTATION);

      double zRotationMetersPerSecond = (Math.toRadians(zRotation) * (Constants.DriveConstants.TRACK_WIDTH / 2)) * -1;

      double leftSpeed = xSpeed - zRotationMetersPerSecond;
      double rightSpeed = xSpeed + zRotationMetersPerSecond;

      double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotationMetersPerSecond));
      double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotationMetersPerSecond));
      if (greaterInput == 0.0) {
        leftSpeed = 0;
        rightSpeed = 0;
      } else {
        double saturatedInput = (greaterInput + lesserInput) / greaterInput;
        leftSpeed /= saturatedInput;
        rightSpeed /= saturatedInput;
      }
      
      wheelDrive(leftSpeed, rightSpeed * -1);
    }

   /**
 * meters per second
 */
  private void wheelDrive(double leftSpeed, double rightSpeed) {
      //60 - converts seconds to minutes;
      //(Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)) gets the wheel circumfrence, otherwise meters per rotation
      //divide by the gear ratio to convert to wheel rotations

      double conversionFactor = (60 / (Math.PI * DriveConstants.WHEEL_DIAMETER)) * Constants.DriveConstants.GEAR_RATIO;

      double leftRPM = leftSpeed * conversionFactor;
      double rightRPM = rightSpeed * conversionFactor;

      SmartDashboard.putNumber("right rpm goal", rightRPM);
     SmartDashboard.putNumber("left rpm goal", leftRPM);

      leftMotorController.setSetpoint(leftRPM * DriveConstants.FORWARD, ControlType.kVelocity);
      rightMotorController.setSetpoint(rightRPM * DriveConstants.FORWARD, ControlType.kVelocity);
  }

  private void pushPIDToSmartDashboard() {
    PID pid = DriveConstants.DRIVE_PID;
    SmartDashboard.putNumber("Drive/P", pid.p);
    SmartDashboard.putNumber("Drive/I", pid.i);
    SmartDashboard.putNumber("Drive/D", pid.d);
    SmartDashboard.putNumber("Drive/kS", pid.kS);
  }

  private void setPID(double p, double i, double d, double kS) {


    if (Math.abs(p  - DriveConstants.DRIVE_PID.p)  < MathConstants.EPSILON &&
      Math.abs(i  - DriveConstants.DRIVE_PID.i)  <  MathConstants.EPSILON &&
      Math.abs(d  - DriveConstants.DRIVE_PID.d)  <  MathConstants.EPSILON &&
      Math.abs(kS - DriveConstants.DRIVE_PID.kS) <  MathConstants.EPSILON) {
      return; // return if there is no meaningful change in PID
    }
 
    PID pid = new PID(p, i, d, kS);
    pid.setStallLimit(DriveConstants.DRIVE_STALL_LIMIT);
    pid.setFreeLimit(DriveConstants.DRIVE_FREE_LIMIT);
    pid.setSparkMaxPID(leftLeaderMotor);  
    pid.setSparkMaxPID(rightLeaderMotor);
    DriveConstants.DRIVE_PID = pid;
  }

  private void updatePID() {
    double p  = SmartDashboard.getNumber("Drive/P",  DriveConstants.DRIVE_PID.p);
    double i  = SmartDashboard.getNumber("Drive/I",  DriveConstants.DRIVE_PID.i);
    double d  = SmartDashboard.getNumber("Drive/D",  DriveConstants.DRIVE_PID.d);
    double kS = SmartDashboard.getNumber("Drive/kS", DriveConstants.DRIVE_PID.kS);

    
    setPID(p, i, d, kS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("right rpm", rightLeaderMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("left rpm", leftLeaderMotor.getEncoder().getVelocity());

    if (RobotState.isTest()) {
      updatePID();
      pushPIDToSmartDashboard();
      
    }
  }

  public void clearIAccum() {
    leftMotorController.setIAccum(0);
    rightMotorController.setIAccum(0);
  }

}
