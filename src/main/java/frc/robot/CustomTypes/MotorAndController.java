package frc.robot.CustomTypes;

import com.revrobotics.spark.SparkClosedLoopController;

public class MotorAndController<MotorType>{

    private MotorType motor;
    private SparkClosedLoopController motorController;
    
    public MotorAndController(MotorType motor, SparkClosedLoopController motorController) {
        this.motor = motor;
        this.motorController = motorController;
    }

    public MotorType getMotor() {return motor;}
    public SparkClosedLoopController getMotorController() {return motorController;}

    public void setMotor(MotorType motor) {this.motor = motor;}
    public void setMotorController(SparkClosedLoopController motorController) {this.motorController = motorController;}

}
