// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * The SetState Command is a very versitile command, it can be used on any state based subsystem to change the state
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CustomTypes.StateBasedSubsystem;


public class SetState <State extends Enum<State>> extends InstantCommand {

  //declare subsystem and the state that it should be set to
  State state;
  StateBasedSubsystem<State> subsystem;

  /**
   * @param subsystem any statebased subsystem
   * @param state the state that to set the subsystem to
   */
  public SetState(StateBasedSubsystem<State> subsystem, State state ) {
    //set global variables
    this.state = state;
    this.subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //use the setState method from StateBasedSubsystem to set the state of the subsystem to the desired state
    subsystem.setState(state);
  }
}
