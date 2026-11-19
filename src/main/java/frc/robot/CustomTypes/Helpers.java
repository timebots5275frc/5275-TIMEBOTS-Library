// This class should hold helper functions that make writing the robot code easier to understand
// These functions should also allow for easier transition between Projects

package frc.robot.CustomTypes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.MotorData;



public class Helpers {
    
    public record MotorAndController<MotorType>(MotorType motor, SparkClosedLoopController motorController){};


    
    /**
     * This function will fully instantiate a motor and its PID
     * @param motor Must be SparkMax; 
     * @param motorController 
     * @param motorData
     * 
     * @return Returns a set of the Motor and its controller; set Motor and Controller values in subsystem equal to its respective value
     */
    public static MotorAndController<SparkMax> InstantiateMotor(SparkMax motor, SparkClosedLoopController motorController, MotorData motorData) {

        motor = new SparkMax(motorData.ID(),SparkLowLevel.MotorType.kBrushless);
        motorData.pid().setFreeLimit(motorData.FREE_LIMIT());
        motorData.pid().setStallLimit(motorData.STALL_LIMIT());
        motorData.pid().setPID(motor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();

        return new MotorAndController<SparkMax>(motor, motorController);
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
}
