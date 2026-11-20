package frc.robot;

import frc.robot.CustomTypes.PID;

public class Constants {
    public record MotorData(int ID, int FREE_LIMIT, int STALL_LIMIT, PID pid){} // defines all constants with a motor

    public static final class IntakeConstants{
        public static final MotorData MOTOR_1_DATA = new MotorData(1,35,60,new PID(0,0,0));
        public static final MotorData MOTOR_2_DATA = new MotorData(2,35,60,new PID(0,0,0));
        public static final int INTAKE_SPEED = 2500;
    }
    
    
}
