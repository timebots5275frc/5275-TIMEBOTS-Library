// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.CustomTypes.Helpers;
import frc.robot.CustomTypes.StateBasedSubsystem;
import frc.robot.CustomTypes.Helpers.MotorAndController;

public class IntakeSubsystem extends StateBasedSubsystem<IntakeSubsystem.IntakeState> {

  public enum IntakeState {
    NONE,
    INTAKE,
    OUTTAKE
  }
  
  private SparkMax intakeMotor1;
  private SparkClosedLoopController intakeController1;
  
  private SparkMax intakeMotor2; 
  private SparkClosedLoopController intakeController2;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
		MotorAndController<SparkMax> m1 = Helpers.InstantiateMotor(intakeMotor1, intakeController1, IntakeConstants.MOTOR_1_DATA);
		MotorAndController<SparkMax> m2 = Helpers.InstantiateMotor(intakeMotor2, intakeController2, IntakeConstants.MOTOR_2_DATA);
		intakeMotor1 = m1.motor();
		intakeController1 = m1.motorController();
		intakeMotor2 = m2.motor();
		intakeController2 = m2.motorController();
  }

	@Override
	protected void changeState(IntakeState state) {
		switch (state) {
			case INTAKE:
				intakeController1.setReference(IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				intakeController2.setReference(IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				break;
			case NONE:
				intakeController1.setReference(0, ControlType.kCurrent);
				intakeController2.setReference(0, ControlType.kCurrent);
				break;
			case OUTTAKE:
				intakeController1.setReference(-IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				intakeController2.setReference(-IntakeConstants.INTAKE_SPEED, ControlType.kVelocity);
				break;
		}
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




}
