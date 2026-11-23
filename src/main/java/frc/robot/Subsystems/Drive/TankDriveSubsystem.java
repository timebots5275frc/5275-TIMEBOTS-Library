package frc.robot.Subsystems.Drive;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.revrobotics.spark.SparkBase.ControlType;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.CustomTypes.Helpers;



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
    //instantaite leaders
    var leftLeader  = Helpers.InstantiateMax(DriveConstants.LEFT_LEADER_DATA);
    var rightLeader = Helpers.InstantiateMax(DriveConstants.RIGHT_LEADER_DATA); 
    //set values
    leftLeaderMotor = leftLeader.getMotor();
    leftMotorController = leftLeader.getMotorController();
    rightLeaderMotor = rightLeader.getMotor();
    rightMotorController = rightLeader.getMotorController();
    // instantiate followers
    leftFollowerMotor  = Helpers.InstantiateFollowerMax(DriveConstants.LEFT_FOLLOWER_DATA, leftLeaderMotor);
    rightFollowerMotor = Helpers.InstantiateFollowerMax(DriveConstants.RIGHT_FOLLOWER_DATA, rightLeaderMotor);

    //make tunable
    DriveConstants.DRIVE_PID.tunable("Drive", leftLeaderMotor, rightLeaderMotor);
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

      //limit x speeds
      xSpeed = Math.min(xSpeed, Constants.DriveConstants.MAX_X_SPEED);
      xSpeed = Math.max(xSpeed, -Constants.DriveConstants.MAX_X_SPEED);
      //limit z speeds
      zRotation = Math.min(zRotation, Constants.DriveConstants.MAX_Z_ROTATION);
      zRotation = Math.max(zRotation, -Constants.DriveConstants.MAX_Z_ROTATION);

      //convert z rotation to meters per second
      double zRotationMetersPerSecond = (Math.toRadians(zRotation) * (Constants.DriveConstants.TRACK_WIDTH / 2)) * -1;

      //calculate left and right speeds
      double leftSpeed = xSpeed - zRotationMetersPerSecond;
      double rightSpeed = xSpeed + zRotationMetersPerSecond;

      //saturate speeds if needed
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
      
      //drive the wheels
      wheelDrive(leftSpeed, rightSpeed * -1);
    }

   /**
    * Drives the robot using individual wheel speeds.
    * @param leftSpeed - meters per second
    * @param rightSpeed - meters per second
    */
  private void wheelDrive(double leftSpeed, double rightSpeed) {
      double leftRPM = leftSpeed * DriveConstants.CONVERSION_FACTOR;
      double rightRPM = rightSpeed * DriveConstants.CONVERSION_FACTOR;

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

    DriveConstants.DRIVE_PID.updateLive();
    
  }

  /**
   * Clears the I accumulator for both left and right motor controllers.
   * This is useful to prevent integral windup when changing setpoints or stopping the robot.
   */
  public void clearIAccum() {
    leftMotorController.setIAccum(0);
    rightMotorController.setIAccum(0);
  }

}
