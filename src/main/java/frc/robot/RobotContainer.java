// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputConstants;
import frc.robot.Commands.SetState;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeState;
import frc.robot.Subsystems.Input.Input;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  IntakeSubsystem intakeSubsystem;

  Joystick joystick;
  XboxController xboxController;
  GenericHID buttonBoard;
  Input input;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SendableChooser<Command> autonChooser) {
    SmartDashboard.putData(autonChooser);

    //Instantiate Subsystems here
    intakeSubsystem = new IntakeSubsystem();
    
    joystick = new Joystick(InputConstants.JOYSTICK_PORT);
    xboxController = new XboxController(InputConstants.XBOX_CONTROLLER_PORT);
    buttonBoard = new GenericHID(InputConstants.BUTTON_BOARD_PORT);
    input = new Input(joystick, xboxController);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //creates a new button
    new JoystickButton(null, 0).onTrue(new SetState<IntakeState>(intakeSubsystem, IntakeState.INTAKE));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(SendableChooser<Command> autonChooser) 
  {

    return autonChooser.getSelected(); 
  }
}