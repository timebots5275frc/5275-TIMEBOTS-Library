package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CustomTypes.MotorData;
import frc.robot.CustomTypes.PID;
import frc.robot.CustomTypes.Math.Vector2;

public class Constants {

  //#region Intake Constants
    /*
      * where all constants for the Intake Subsystem should be defined
      */
    public static final class IntakeConstants{
      public static final MotorData MOTOR_1_DATA = new MotorData(1,35,60,new PID(0,0,0));
      public static final MotorData MOTOR_2_DATA = new MotorData(2,35,60,new PID(0,0,0));
      public static final int INTAKE_SPEED = 2500;
    }

    //#region Button Constants
    /*
      * this is where button ID contstants should be defined
      */
    public static final class ButtonConstants{
        
    }

    //#region Controller Constants
    public static final class ControllerConstants {
      public static final int XBOX_CONTROLLER_PORT = 0;
      public static final int JOYSTICK_PORT = 1;
      public static final double DEADZONE_DRIVE    = 0.1;
      public static final double DEADZONE_STEER    = 0.3;
    }

    //#region Input Constants
    public class InputConstants {

      public static final int JOYSTICK_PORT = 0;
      public static final int XBOX_CONTROLLER_PORT = 0;
      public static final int BUTTON_BOARD_PORT = 1;

    }

    //#region Drive constants
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
      public static final double TRACK_WIDTH = 14.5 * MathConstants.INCH_TO_METER; 
      public static final double GEAR_RATIO = 10.71; 
      public static final double WHEEL_DIAMETER = 6 * MathConstants.INCH_TO_METER; 
      public static final double CONVERSION_FACTOR = (60 / (Math.PI * DriveConstants.WHEEL_DIAMETER)) * Constants.DriveConstants.GEAR_RATIO;// meters per second to rotations per minute
        
    }

    //#region Math Constants
    public static final class MathConstants
    {
      public static final double INCH_TO_METER = 0.0254;
      public static final double EPSILON = 1e-6;
    }

    //#region Vision Constants
    public static final class VisionConstants {
      
      //#region April Tag 
      public static enum AprilTagData
      {
        //Red Hub
        Hub_Right_Far_Red(2, "Hub Right Far", DriverStation.Alliance.Red),
        Hub_Back_Right_Red(3, "Hub Back Right", DriverStation.Alliance.Red),
        Hub_Back_Left_Red(4, "Hub Back Left", DriverStation.Alliance.Red, 0, 16.6193978),
        Hub_Left_Far_Red(5, "Hub Left Far", DriverStation.Alliance.Red, -2.7389074, 14.778355),
        Hub_Left_Close_Red(8, "Hub Left Close", DriverStation.Alliance.Red),
        Hub_Front_Left_Red(9, "Hub Front Left", DriverStation.Alliance.Red),
        Hub_Front_Right_Red(10, "Hub Front Right", DriverStation.Alliance.Red),
        Hub_Right_Close_Red(11, "Hub Right Close", DriverStation.Alliance.Red),
        //Blue Hub
        Hub_Right_Far_Blue(18, "Hub Right Far", DriverStation.Alliance.Blue),
        Hub_Back_Right_Blue(19, "Hub Back Right", DriverStation.Alliance.Blue),
        Hub_Back_Left_Blue(20, "Hub Back Left", DriverStation.Alliance.Blue, 0, 16.6193978),
        Hub_Left_Far_Blue(21, "Hub Left Far", DriverStation.Alliance.Blue, -2.7389074, 14.778355),
        Hub_Left_Close_Blue(24, "Hub Left Close", DriverStation.Alliance.Blue),
        Hub_Front_Left_Blue(25, "Hub Front Left", DriverStation.Alliance.Blue),
        Hub_Front_Right_Blue(26, "Hub Front Right", DriverStation.Alliance.Blue),
        Hub_Right_Close_Blue(27, "Hub Right Close", DriverStation.Alliance.Blue);

        public final int id;
        public final String name;
        public final DriverStation.Alliance alliance;
        private AprilTagData(int id, String name, DriverStation.Alliance alliance)
        {
          this.id = id;
          this.name = name;
          this.alliance = alliance;
        }

        private AprilTagData(int id, String name, DriverStation.Alliance alliance, double x, double y)
        {
          this.id = id;
          this.name = name;
          this.alliance = alliance;
        }

        @Override
        public String toString()
        {
          return (alliance == DriverStation.Alliance.Blue ? "Blue " : "Red ") + name + "[" + id + "]";
        }

        public static AprilTagData getTag(int id) {
          switch(id){
            // case 1: return Trench_Right_Neutral_Zone;
            case 2: return Hub_Right_Far_Red;
            case 3: return Hub_Back_Right_Red;
            case 4: return Hub_Back_Left_Red;
            case 5: return Hub_Left_Far_Red;
            // case 6: return Trench_Left_Neutral_Zone;
            // case 7: return Trench_Left_Alliance_Zone;
            case 8: return Hub_Left_Close_Red;
            case 9: return Hub_Front_Left_Red;
            case 10: return Hub_Front_Right_Red;
            case 11: return Hub_Right_Close_Red;
            // case 12: return Trench_Right_Alliance_Zone;
            // case 13: return Loading_Station_Right;
            // case 14: return Loading_Station_Left;
            // case 15: return Tower_Right;
            // case 16: return Tower_Left;
            // case 17: return Trench_Right_Neutral_Zone;
            case 18: return Hub_Right_Far_Blue;
            case 19: return Hub_Back_Right_Blue;
            case 20: return Hub_Back_Left_Blue;
            case 21: return Hub_Left_Far_Blue;
            // case 22: return ba_amplifier;
            // case 23: return ba_speaker_main;
            case 24: return Hub_Left_Close_Blue;
            case 25: return Hub_Front_Left_Blue;
            case 26: return Hub_Front_Right_Blue;
            case 27: return Hub_Right_Close_Blue;
            // case 28: return ra_core_opp_scoring_table;
            // case 29: return ra_core_mid;
            // case 30: return ba_core_mid;
            // case 31: return ba_core_opp_scoring_table;
            // case 32: return ba_core_scoring_table;
            default: return null;
          }
        }
      }

    public static final int VALUES_TO_AVERAGE = 0;
    }
    //#endregion
}
