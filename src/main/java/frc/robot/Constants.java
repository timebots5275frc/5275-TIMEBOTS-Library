package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public class VisionConstants {

      public static final int VALUES_TO_AVERAGE = 5; // Number of values to average for smoothing the robot's position and rotation estimates
      


      public static final double HUB_FIELD_SPACE_Z_POSITION = 1.8288;
      public static final double RED_HUB_FIELD_SPACE_X_POSITION = 11.919;
      public static final double RED_HUB_FIELD_SPACE_Y_POSITION = 4.02844;
      public static final double BLUE_HUB_FIELD_SPACE_X_POSITION = 4.626; 
      public static final double BLUE_HUB_FIELD_SPACE_Y_POSITION = 4.02844; 
      public static final Translation3d RED_HUB_POSE = new Translation3d(RED_HUB_FIELD_SPACE_X_POSITION, RED_HUB_FIELD_SPACE_Y_POSITION,HUB_FIELD_SPACE_Z_POSITION);
      public static final Translation3d BLUE_HUB_POSE = new Translation3d(BLUE_HUB_FIELD_SPACE_X_POSITION, BLUE_HUB_FIELD_SPACE_Y_POSITION,HUB_FIELD_SPACE_Z_POSITION);


      public class AprilTagFieldConstants {

      public static final double FIELD_LENGTH = 16.54;
      public static final double FIELD_WIDTH = 8.07;

      public static final List<AprilTag> TAGS = List.of(
      new AprilTag(1, new Pose3d(
        new Translation3d(11.878, 7.425, 0.889),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(2, new Pose3d(
        new Translation3d(11.915, 4.638, 1.124),
        new Rotation3d(0, 0, Math.toRadians(90)))),

      new AprilTag(3, new Pose3d(
        new Translation3d(11.312, 4.390, 1.124),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(4, new Pose3d(
        new Translation3d(11.312, 4.035, 1.124),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(5, new Pose3d(
        new Translation3d(11.915, 3.431, 1.124),
        new Rotation3d(0, 0, Math.toRadians(270)))),

      new AprilTag(6, new Pose3d(
        new Translation3d(11.878, 0.644, 0.889),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(7, new Pose3d(
        new Translation3d(11.953, 0.644, 0.889),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(8, new Pose3d(
        new Translation3d(12.271, 3.431, 1.124),
        new Rotation3d(0, 0, Math.toRadians(270)))),

      new AprilTag(9, new Pose3d(
        new Translation3d(4.022, 3.679, 1.124),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(10, new Pose3d(
        new Translation3d(12.519, 4.035, 1.124),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(11, new Pose3d(
        new Translation3d(12.271, 4.638, 1.124),
        new Rotation3d(0, 0, Math.toRadians(90)))),

      new AprilTag(12, new Pose3d(
        new Translation3d(11.953, 7.425, 0.889),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(13, new Pose3d(
        new Translation3d(16.533, 7.403, 0.552),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(14, new Pose3d(
        new Translation3d(16.533, 6.972, 0.552),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(15, new Pose3d(
        new Translation3d(16.533, 4.324, 0.552),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(16, new Pose3d(
        new Translation3d(16.533, 3.892, 0.552),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(17, new Pose3d(
        new Translation3d(4.663, 0.644, 0.889),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(18, new Pose3d(
        new Translation3d(4.626, 3.431, 1.124),
        new Rotation3d(0, 0, Math.toRadians(270)))),

      new AprilTag(19, new Pose3d(
        new Translation3d(5.229, 3.679, 1.124),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(20, new Pose3d(
        new Translation3d(5.229, 4.035, 1.124),
        new Rotation3d(0, 0, Math.toRadians(90)))),

      new AprilTag(21, new Pose3d(
        new Translation3d(4.626, 4.638, 1.124),
        new Rotation3d(0, 0, Math.toRadians(90)))),

      new AprilTag(22, new Pose3d(
        new Translation3d(4.663, 7.425, 0.889),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(23, new Pose3d(
        new Translation3d(4.588, 7.425, 0.889),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(24, new Pose3d(
        new Translation3d(4.271, 4.638, 1.124),
        new Rotation3d(0, 0, Math.toRadians(90)))),

      new AprilTag(25, new Pose3d(
        new Translation3d(4.022, 4.390, 1.124),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(26, new Pose3d(
        new Translation3d(4.022, 4.035, 1.124),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(27, new Pose3d(
        new Translation3d(4.271, 3.431, 1.124),
        new Rotation3d(0, 0, Math.toRadians(270)))),

      new AprilTag(28, new Pose3d(
        new Translation3d(4.588, 0.644, 0.889),
        new Rotation3d(0, 0, Math.toRadians(180)))),

      new AprilTag(29, new Pose3d(
        new Translation3d(0.008, 0.666, 0.552),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(30, new Pose3d(
        new Translation3d(0.008, 1.098, 0.552),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(31, new Pose3d(
        new Translation3d(0.008, 3.746, 0.552),
        new Rotation3d(0, 0, Math.toRadians(0)))),

      new AprilTag(32, new Pose3d(
        new Translation3d(0.008, 4.178, 0.552),
        new Rotation3d(0, 0, Math.toRadians(0))))
      );
        }
    }
    //#endregion
}
