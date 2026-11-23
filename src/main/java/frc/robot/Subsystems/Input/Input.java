package frc.robot.Subsystems.Input;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.CustomTypes.Math.Vector2;

/**
 * Single access point for driver input from a flight joystick AND an Xbox controller.
 *
 * Whichever device most recently produced non-zero input is the "active" device.
 * Driving code should only use the getDrive* / get*SpeedPercent getters, which
 * always return values from the active device. Device-specific getters are also
 * available when you need them.
 *
 * Axes: x = forward (+), y = left (+). Turn is passed through as the device reports it.
 * All values are updated once per loop in periodic(); getters never touch hardware.
 */
public class Input extends SubsystemBase {

  /** How much one button press changes the Xbox controller's speed limit (0-1 scale). */
  private static final double CONTROLLER_SPEED_STEP = 0.2;

  private final Joystick joystick;
  private final XboxController controller;

  // ---- cached, deadbanded values (updated in periodic) ----
  private Vector2 joystickMove = Vector2.zero;
  private double joystickTurn = 0;
  private double joystickSpeedPercent = 0;   // from the throttle slider, 0-1

  private Vector2 controllerMove = Vector2.zero;
  private double controllerTurn = 0;

  // Controller has no slider, so its speed limit is adjusted with buttons.
  private double controllerTranslationSpeed = 0.6;
  private double controllerTwistSpeed = 0.6;

  private boolean usingJoystick = true;

  /**
   * Creates the input subsystem.
   *
   * @param joystick   the flight joystick used for driving
   * @param controller the Xbox controller used for driving and rumble feedback
   */
  public Input(Joystick joystick, XboxController controller) {
    this.joystick = joystick;
    this.controller = controller;
  }

  /**
   * Runs once per scheduler loop. Reads both devices, then decides which one is active:
   * the joystick if it has input, otherwise the controller if it has input, otherwise
   * whichever was active last.
   */
  @Override
  public void periodic() {
    readJoystick();
    readController();

    // Joystick wins if both are active at once; otherwise stay on the last active device.
    if (hasJoystickInput()) {
      usingJoystick = true;
    } else if (hasControllerInput()) {
      usingJoystick = false;
    }
  }

  //#region Reading hardware

  /**
   * Reads the joystick axes, applies deadbands, and converts the throttle slider
   * to a 0-1 speed percent. Stores the results in the cached joystick fields.
   */
  private void readJoystick() {
    joystickMove = new Vector2(
        deadband(-joystick.getY(), ControllerConstants.DEADZONE_DRIVE),
        deadband(-joystick.getX(), ControllerConstants.DEADZONE_DRIVE));
    joystickTurn = deadband(joystick.getTwist(), ControllerConstants.DEADZONE_STEER);
    // slider reads -1 (pushed forward) to +1 (pulled back); convert to 0-1 where forward = full speed
    joystickSpeedPercent = (1 - joystick.getThrottle()) / 2;
  }

  /**
   * Reads the Xbox controller sticks (left = move, right X = turn), applies deadbands,
   * and stores the results in the cached controller fields.
   */
  private void readController() {
    controllerMove = new Vector2(
        deadband(-controller.getLeftY(), ControllerConstants.DEADZONE_DRIVE),
        deadband(-controller.getLeftX(), ControllerConstants.DEADZONE_DRIVE));
    controllerTurn = deadband(controller.getRightX(), ControllerConstants.DEADZONE_STEER);
  }

  /**
   * Zeroes values inside the deadzone and rescales the rest so output still
   * ramps smoothly from 0 to +/-1 (no jump at the edge of the deadzone).
   *
   * @param value    raw axis value, -1 to 1
   * @param deadzone size of the deadzone, 0 to 1
   * @return the deadbanded value, -1 to 1
   */
  private static double deadband(double value, double deadzone) {
    return MathUtil.applyDeadband(value, deadzone);
  }

   /**
   * Squares a value while keeping its sign (e.g. -0.5 becomes -0.25).
   *
   * @param value input value, -1 to 1
   * @return the squared value with the original sign, -1 to 1
   */
  private static double square(double value) {
    return Math.copySign(value * value, value);
  }
  //#endregion

  //#region Driving getters (from whichever device is active)

  /**
   * @return move input (x forward, y left) from whichever device is active, each -1 to 1
   */
  public Vector2 getDriveInput() { return usingJoystick ? joystickMove : controllerMove; }

  /**
   * @return turn input from whichever device is active, -1 to 1
   */
  public double getDriveTurn() { return usingJoystick ? joystickTurn : controllerTurn; }

  /**
   * @return translation speed limit (0-1) from the active device: the throttle slider
   *         for the joystick, or the button-adjusted value for the controller
   */
  public double getTranslationSpeedPercent() {
    return usingJoystick ? joystickSpeedPercent : controllerTranslationSpeed;
  }

  /**
   * @return turn speed limit (0-1) from the active device: the throttle slider
   *         for the joystick, or the button-adjusted value for the controller
   */
  public double getTwistSpeedPercent() {
    return usingJoystick ? joystickSpeedPercent : controllerTwistSpeed;
  }

  /**
   * @return the active device's move input already multiplied by the translation speed limit
   */
  public Vector2 getScaledDriveInput() {
    Vector2 move = getDriveInput();
    double scale = getTranslationSpeedPercent();
    return new Vector2(move.x * scale, move.y * scale);
  }

  /**
   * @return the active device's turn input already multiplied by the twist speed limit
   */
  public double getScaledDriveTurn() { return getDriveTurn() * getTwistSpeedPercent(); }

  /**
   * Squared inputs give finer control at low stick deflection while still reaching full
   * output at full deflection. Sign (direction) is preserved.
   *
   * @return the active device's move input with each component squared, -1 to 1
   */
  public Vector2 getSquaredDriveInput() {
    Vector2 move = getDriveInput();
    return new Vector2(square(move.x), square(move.y));
  }

  /**
   * @return the active device's turn input squared (sign preserved), -1 to 1
   */
  public double getSquaredDriveTurn() { return square(getDriveTurn()); }

  /**
   * Squares the move input first, then applies the translation speed limit, so the
   * speed limit stays the true maximum output.
   *
   * @return the active device's squared move input multiplied by the translation speed limit
   */
  public Vector2 getSquaredScaledDriveInput() {
    Vector2 move = getSquaredDriveInput();
    double scale = getTranslationSpeedPercent();
    return new Vector2(move.x * scale, move.y * scale);
  }

  /**
   * Squares the turn input first, then applies the twist speed limit.
   *
   * @return the active device's squared turn input multiplied by the twist speed limit
   */
  public double getSquaredScaledDriveTurn() { return getSquaredDriveTurn() * getTwistSpeedPercent(); }

  /**
   * @return true if the active device is the joystick, false if it is the Xbox controller
   */
  public boolean isUsingJoystick() { return usingJoystick; }

  /**
   * @return true if the active device is currently giving any move or turn input
   */
  public boolean hasDriveInput() { return usingJoystick ? hasJoystickInput() : hasControllerInput(); }
  //#endregion

  //#region Device-specific getters

  /**
   * @return the joystick's move input (x forward, y left), regardless of which device is active
   */
  public Vector2 getJoystickInput() { return joystickMove; }

  /**
   * @return the joystick's twist input, regardless of which device is active
   */
  public double getJoystickTurn() { return joystickTurn; }

  /**
   * @return the joystick throttle slider as a 0-1 speed percent (pushed forward = 1)
   */
  public double getJoystickSpeedPercent() { return joystickSpeedPercent; }

  /**
   * @return true if the joystick is outside its deadzones on move or twist
   */
  public boolean hasJoystickInput() {
    return joystickMove.x != 0 || joystickMove.y != 0 || joystickTurn != 0;
  }

  /**
   * @return the Xbox controller's move input (left stick; x forward, y left),
   *         regardless of which device is active
   */
  public Vector2 getControllerInput() { return controllerMove; }

  /**
   * @return the Xbox controller's turn input (right stick X), regardless of which device is active
   */
  public double getControllerTurn() { return controllerTurn; }

  /**
   * @return true if the Xbox controller is outside its deadzones on move or turn
   */
  public boolean hasControllerInput() {
    return controllerMove.x != 0 || controllerMove.y != 0 || controllerTurn != 0;
  }
  //#endregion

  //#region Controller speed limits (button-adjusted)

  /**
   * @return the Xbox controller's current translation speed limit, 0-1
   */
  public double getControllerTranslationSpeed() { return controllerTranslationSpeed; }

  /**
   * @return the Xbox controller's current turn speed limit, 0-1
   */
  public double getControllerTwistSpeed() { return controllerTwistSpeed; }

  /** Raises the controller's translation speed limit by one step, capped at 1. */
  public void incrementControllerTranslationSpeed() {
    controllerTranslationSpeed = stepSpeed(controllerTranslationSpeed, CONTROLLER_SPEED_STEP);
  }

  /** Lowers the controller's translation speed limit by one step, floored at 0. */
  public void decrementControllerTranslationSpeed() {
    controllerTranslationSpeed = stepSpeed(controllerTranslationSpeed, -CONTROLLER_SPEED_STEP);
  }

  /** Raises the controller's turn speed limit by one step, capped at 1. */
  public void incrementControllerTwistSpeed() {
    controllerTwistSpeed = stepSpeed(controllerTwistSpeed, CONTROLLER_SPEED_STEP);
  }

  /** Lowers the controller's turn speed limit by one step, floored at 0. */
  public void decrementControllerTwistSpeed() {
    controllerTwistSpeed = stepSpeed(controllerTwistSpeed, -CONTROLLER_SPEED_STEP);
  }

  /**
   * Adds a step to a speed limit, clamps to 0-1, and rounds to 2 decimals so repeated
   * 0.2 steps don't drift (e.g. 0.6000000000000001).
   *
   * @param current the current speed limit
   * @param step    amount to add (negative to decrease)
   * @return the new speed limit, 0-1
   */
  private static double stepSpeed(double current, double step) {
    return Math.round(MathUtil.clamp(current + step, 0, 1) * 100) / 100.0;
  }
  //#endregion

  //#region Rumble (Xbox controller)

  /**
   * Sets rumble on the Xbox controller.
   *
   * @param type     which motor(s) to rumble (left, right, or both)
   * @param strength rumble strength, 0 to 1
   */
  public void rumbleController(GenericHID.RumbleType type, double strength) {
    controller.setRumble(type, strength);
  }

  /** Turns both rumble motors on at full strength. Call stopRumble() to turn them off. */
  public void flipRumble() { rumbleController(GenericHID.RumbleType.kBothRumble, 1); }

  /** Turns both rumble motors off. */
  public void stopRumble() { rumbleController(GenericHID.RumbleType.kBothRumble, 0); }
  //#endregion
}