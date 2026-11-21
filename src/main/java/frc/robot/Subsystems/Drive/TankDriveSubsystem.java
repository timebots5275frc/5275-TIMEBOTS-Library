package frc.robot.Subsystems.Drive;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.revrobotics.spark.SparkBase.ControlType;

import java.util.HashMap;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.CustomTypes.Helpers;
import frc.robot.CustomTypes.MotorAndController;
import frc.robot.CustomTypes.MotorData;
import frc.robot.CustomTypes.PID;


public class TankDriveSubsystem extends SubsystemBase {

    private SparkMax leftLeaderMotor;
    private SparkMax rightLeaderMotor;

    private SparkMax leftFollowerMotor;
    private SparkMax rightFollowerMotor;

    private SparkClosedLoopController leftMotorController;
    private SparkClosedLoopController rightMotorController;

    private HashMap<MotorAndController<SparkMax>,MotorData> motor_controller_data = new HashMap<MotorAndController<SparkMax>,MotorData>();

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
    leftLeaderMotor = leftLeader.getMotor();
    leftMotorController = leftLeader.getMotorController();

    //right leader
    MotorAndController<SparkMax> rightLeader = Helpers.InstantiateMotor(rightLeaderMotor, rightMotorController, DriveConstants.LEFT_LEADER_DATA);
    rightLeaderMotor = rightLeader.getMotor();
    rightMotorController = rightLeader.getMotorController();

    //left follower
    leftFollowerMotor = Helpers.InstantiateFollowerMotor(leftFollowerMotor, leftLeaderMotor, DriveConstants.LEFT_FOLLOWER_DATA);
    
    //right follower
    rightFollowerMotor = Helpers.InstantiateFollowerMotor(rightFollowerMotor, rightLeaderMotor, DriveConstants.RIGHT_FOLLOWER_DATA);

    motor_controller_data.put(rightLeader, DriveConstants.LEFT_LEADER_DATA);
    motor_controller_data.put(leftLeader, DriveConstants.RIGHT_LEADER_DATA);
  }

  /**
   *  
   * @param xSpeed 
   * meters per second
   * @param zRotation
   * degrees per second, positive is Counter Clockwise
   */
    public void arcadeDrive(double xSpeed, double zRotation) {


      zRotation *= -1; // reverse rotation


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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //put motor velo data
    SmartDashboard.putNumber("right rpm", rightLeaderMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("left rpm", leftLeaderMotor.getEncoder().getVelocity());

    DriveConstants.DRIVE_PID.updateLivePIDMax(motor_controller_data);
    
  }

  public void clearIAccum() {
    leftMotorController.setIAccum(0);
    rightMotorController.setIAccum(0);
  }

}
