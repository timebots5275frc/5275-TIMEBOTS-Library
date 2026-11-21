package frc.robot;

import frc.robot.CustomTypes.MotorData;
import frc.robot.CustomTypes.PID;

public class Constants {
    /*
     * where all constants for the Intake Subsystem should be defined
     */
    public static final class IntakeConstants{
        public static final MotorData MOTOR_1_DATA = new MotorData(1,35,60,new PID(0,0,0));
        public static final MotorData MOTOR_2_DATA = new MotorData(2,35,60,new PID(0,0,0));
        public static final int INTAKE_SPEED = 2500;
    }
    
    /*
     * this is where button ID contstants should be defined
     */
    public static final class ButtonConstants{
       
    }

    public static final class ControllerConstants {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL    = 1;
      public static final double DEADZONE_DRIVE    = 0.1;
      public static final double DEADZONE_STEER    = 0.3;
    }

    public static final class JoystickConstants
    {
      public static final double JOY_X_RATE_LIMIT = 6;
      public static final double JOY_TURN_RATE_LIMIT = 12;

      public static final double JOY_INPUT_VELOCITY_MULT = 1;
      public static final double JOY_INPUT_ROTATION_VELOCITY_MULT = 1;
    }

    public static class DriveConstants {
        
        public static  PID DRIVE_PID = new PID(0.00015,0.0000005,0.00013,0); //TODO: PID TUNING
        public static final int FORWARD = 1; //flip sign to flip foward

        //current limits
        public static final int DRIVE_FREE_LIMIT = 45; 
        public static final int DRIVE_STALL_LIMIT = 60; 

        //define motor data
        public static final MotorData LEFT_LEADER_DATA = new MotorData(2, DRIVE_FREE_LIMIT, DRIVE_STALL_LIMIT, DRIVE_PID);
        public static final MotorData LEFT_FOLLOWER_DATA = new MotorData(4, DRIVE_FREE_LIMIT, DRIVE_STALL_LIMIT, DRIVE_PID);
        public static final MotorData RIGHT_LEADER_DATA = new MotorData(3, DRIVE_FREE_LIMIT, DRIVE_STALL_LIMIT, DRIVE_PID);
        public static final MotorData RIGHT_FOLLOWER_DATA = new MotorData(5, DRIVE_FREE_LIMIT, DRIVE_STALL_LIMIT, DRIVE_PID);


        //limits
        public static final double MAX_X_SPEED = 1.5; // meters per second
        public static final double MAX_Z_ROTATION = 180; // degrees per second

        //geomotry
        public static final double TRACK_WIDTH = 14.5 * MathConstants.INCH_TO_METER; //TODO: the distance between the middle of left and right wheel
        public static final double GEAR_RATIO = 10.71; //TODO: how many rotations of the motor rotate the wheel
        public static final double WHEEL_DIAMETER = 6 * MathConstants.INCH_TO_METER; //TODO:

       
    }

  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
    public static final double EPSILON = 1e-6;
  }
    
}
