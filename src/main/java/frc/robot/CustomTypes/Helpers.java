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


    //TODO: merge these two functions together using generics; will have to merge SparkMax and SparkFlex in PID class using genarics too
    /**
     * This function will fully instantiate A SparkMax motor and its PID
     * @param motor Must be SparkMax; 
     * @param motorController 
     * @param motorData
     * 
     * @return Returns a set of the Motor and its controller; set Motor and Controller values in subsystem equal to its respective value
     */
    public static MotorAndController<SparkMax> InstantiateSparkMax(SparkMax motor, SparkClosedLoopController motorController, MotorData motorData) {

        motor = new SparkMax(motorData.ID(),SparkLowLevel.MotorType.kBrushless);
        motorData.pid().setFreeLimit(motorData.FREE_LIMIT());
        motorData.pid().setStallLimit(motorData.STALL_LIMIT());
        motorData.pid().setSparkMaxPID(motor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();

        return new MotorAndController<SparkMax>(motor, motorController);
    }

    /**
     * This function will fully instantiate A SparkFlex motor and its PID; Make sure to set values with output of this function
     * @param motor Must be SparkFlex; 
     * @param motorController 
     * @param motorData
     * 
     * @return Returns a set of the Motor and its controller; set Motor and Controller values in subsystem equal to its respective value
     */
    public static MotorAndController<SparkFlex> InstantiateSparkFlex(SparkFlex motor, SparkClosedLoopController motorController, MotorData motorData) {

        motor = new SparkFlex(motorData.ID(),SparkLowLevel.MotorType.kBrushless);
        motorData.pid().setFreeLimit(motorData.FREE_LIMIT());
        motorData.pid().setStallLimit(motorData.STALL_LIMIT());
        motorData.pid().setSparkFlexPID(motor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();
        return new MotorAndController<SparkFlex>(motor, motorController);
    }
}
