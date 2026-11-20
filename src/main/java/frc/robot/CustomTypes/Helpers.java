// This class should hold helper functions that make writing the robot code easier to understand
// These functions should also allow for easier transition between Projects

package frc.robot.CustomTypes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.MotorData;



public class Helpers {
    //define an object that contains the motor and its controller, to allow for both to be returned from functions at the same time
    public record MotorAndController<MotorType>(MotorType motor, SparkClosedLoopController motorController){};


    //#region Motor Instantiation
    /**
     * This function will fully instantiate a motor and its PID
     * @param motor
     * @param motorController 
     * @param motorData
     * 
     * @return Returns a set of the Motor and its controller; set Motor and Controller values in subsystem equal to its respective value
     */
    public static MotorAndController<SparkMax> InstantiateMotor(SparkMax motor, SparkClosedLoopController motorController, MotorData motorData) {

        motor = new SparkMax(motorData.ID(),SparkLowLevel.MotorType.kBrushless);//instantiate motor
        motorData.pid().setFreeLimit(motorData.FREE_LIMIT());//set free limit
        motorData.pid().setStallLimit(motorData.STALL_LIMIT());//set stall limit
        motorData.pid().setPID(motor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);//set the PID
        motorController = motor.getClosedLoopController();//set the controller

        return new MotorAndController<SparkMax>(motor, motorController);//return the motor + controller
    }

    /**
     * This function will fully instantiate a motor and its PID
     * @param motor 
     * @param motorController 
     * @param motorData
     * 
     * @return Returns a set of the Motor and its controller; set Motor and Controller values in subsystem equal to its respective value
     */
    public static MotorAndController<SparkFlex> InstantiateMotor(SparkFlex motor, SparkClosedLoopController motorController, MotorData motorData) {

        motor = new SparkFlex(motorData.ID(),SparkLowLevel.MotorType.kBrushless);
        motorData.pid().setFreeLimit(motorData.FREE_LIMIT());
        motorData.pid().setStallLimit(motorData.STALL_LIMIT());
        motorData.pid().setPID(motor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();
        return new MotorAndController<SparkFlex>(motor, motorController);
    }


     /**
     * This function will fully instantiate a motor and its PID
     * @param motor - motor that will follow the leader
     * @param leaderMotor - motor that leads the follower 
     * @param motorData
     * 
     * @return returns motor that follows leader motor
     */
    public static SparkMax InstantiateFollowerMotor(SparkMax motor, SparkMax leaderMotor, MotorData motorData) {
        motor = new SparkMax(motorData.ID(), SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config1 = motorData.pid().setPID(leaderMotor, IdleMode.kCoast);
        config1.follow(leaderMotor, false); 
        motor.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        return motor;
    }

    /**
     * This function will fully instantiate a motor and its PID
     * @param motor - motor that will follow the leader
     * @param leaderMotor - motor that leads the follower 
     * @param motorData
     * 
     * @return returns motor that follows leader motor
     */
    public static SparkFlex InstantiateFollowerMotor(SparkFlex motor, SparkFlex leaderMotor, MotorData motorData) {
        motor = new SparkFlex(motorData.ID(), SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig config1 = motorData.pid().setPID(leaderMotor, IdleMode.kCoast);
        config1.follow(leaderMotor, false); 
        motor.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        return motor;
    }

    //#endregion Motor Instantiation
}
