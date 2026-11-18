// Most Subsystems should extend this class
// this allows for every state based class to share the same Command to change state, reducing 1+ command for each subsytem to change state, to one for all

package frc.robot.CustomTypes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateBasedSubsystem<State extends Enum<State>> extends SubsystemBase {

  protected State state;

  

  /** Creates a new StateBasedSubsystem. */
  public StateBasedSubsystem() {

  }

  /**
   * @return returns the current state of the subsystem
   */
  public State getState() {
    return state;
  }

  /**
   * @param state sets the state of the subsystem & calls changeState
   */
  public void setState(State state) {
    this.state = state;
  }

  protected abstract void changeState(State state);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
