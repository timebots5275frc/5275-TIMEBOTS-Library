// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CustomTypes.StateBasedSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetState <State extends Enum<State>> extends InstantCommand {
  State state;
  StateBasedSubsystem<State> subsystem;

  /**
   * 
   * @param subsystem any statebased subsystem
   * @param state the state that to set the subsystem to
   */
  public SetState(StateBasedSubsystem<State> subsystem, State state ) {
    this.state = state;
    this.subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setState(state);
  }
}
