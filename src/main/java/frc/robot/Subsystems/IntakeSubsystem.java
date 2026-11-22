// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * this class is should be used as a starting point in designing an intake
 * This Subsystem assumes 2 intake motors that spin in the same direction for intake and outtake
 * 
 * This subsystem extends from StateBasedSubsystem, allowing for usage of SetState Command
 */

package frc.robot.Subsystems;

import java.util.HashMap;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.CustomTypes.Helpers;
import frc.robot.CustomTypes.MotorAndController;
import frc.robot.CustomTypes.MotorData;
import frc.robot.CustomTypes.StateBasedSubsystem;


//Extends StateBasedSubsystem, because it operates on states, as described by the enum; pass the enum type, IntakeState, into the <>
public class IntakeSubsystem extends StateBasedSubsystem<IntakeSubsystem.IntakeState> {

	//create Enum describing possible states for the Intake
  public enum IntakeState {
    NONE,
    INTAKE,
    OUTTAKE 
  }
  
  //Declare motor 1 & controller
  private SparkMax intakeMotor1;
  private SparkClosedLoopController intakeController1;
  //Declare motor 2 & controller
  private SparkMax intakeMotor2; 
  private SparkClosedLoopController intakeController2;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

	//get a set with the motor and controller for motor 1
	MotorAndController<SparkMax> m1 = Helpers.InstantiateMax(IntakeConstants.MOTOR_1_DATA);
	//get a set with the motor and controller for motor 2
	MotorAndController<SparkMax> m2 = Helpers.InstantiateMax(IntakeConstants.MOTOR_2_DATA);

	//set values for motor 1 and controller 1
	intakeMotor1 = m1.getMotor();
	intakeController1 = m1.getMotorController();
	//set values for motor 2 and controller 2
	intakeMotor2 = m2.getMotor();
	intakeController2 = m2.getMotorController();

	IntakeConstants.MOTOR_1_DATA.getPID().tunable("Intake 1", intakeMotor1);
	IntakeConstants.MOTOR_2_DATA.getPID().tunable("Intake 2", intakeMotor2);

  }

	@Override
	protected void changeState(IntakeState state) {
		//automatically called through setState() method in the parent class, StateBasedSubsystem

		switch (state) {//based on state, set motor speeds
			case INTAKE://both motors run forward at intake speed
				intakeController1.setSetpoint(IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				intakeController2.setSetpoint(IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				break;
			case NONE://both motors get 0 Current; IMPORTANT to use kCurrent
				intakeController1.setSetpoint(0, ControlType.kCurrent);
				intakeController2.setSetpoint(0, ControlType.kCurrent);
				break;
			case OUTTAKE://Both motors spin backwards
				intakeController1.setSetpoint(-IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				intakeController2.setSetpoint(-IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				break;
		}
	}

  @Override
  public void periodic() {
	//Log Intake State
	SmartDashboard.putString("State",state.name());

	//Log motor 1 data
    SmartDashboard.putNumber("Intake Motor 1 RPM", intakeMotor1.getEncoder().getVelocity());
	SmartDashboard.putNumber("Intake Motor 1 Current", intakeMotor1.getOutputCurrent());

	//Log motor 2 data
    SmartDashboard.putNumber("Intake Motor 2 RPM", intakeMotor2.getEncoder().getVelocity());
	SmartDashboard.putNumber("Intake Motor 2 Current", intakeMotor2.getOutputCurrent());
	
	//update live pid
	IntakeConstants.MOTOR_1_DATA.getPID().updateLive();
	IntakeConstants.MOTOR_2_DATA.getPID().updateLive();
  }

}
