// Helper functions that make writing the robot code easier to understand
// and easier to move between projects.

package frc.robot.CustomTypes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Helpers {

    //#region Motor Instantiation

    /**
     * Creates and fully configures a SparkMax (ID, current limits, idle mode, PID/FF).
     * Usage:  var m = Helpers.InstantiateMax(DriveConstants.LEFT_LEADER_DATA); 
     *         motor = m.getMotor();  controller = m.getMotorController();
     */
    public static MotorAndController<SparkMax> InstantiateMax(MotorData data) {
        SparkMax motor = new SparkMax(data.getID(), SparkLowLevel.MotorType.kBrushless);
        motor.configure(data.applyTo(new SparkMaxConfig()), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return new MotorAndController<SparkMax>(motor, motor.getClosedLoopController());
    }

    /** Same as InstantiateMax, for a SparkFlex. */
    public static MotorAndController<SparkFlex> InstantiateFlex(MotorData data) {
        SparkFlex motor = new SparkFlex(data.getID(), SparkLowLevel.MotorType.kBrushless);
        motor.configure(data.applyTo(new SparkFlexConfig()), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return new MotorAndController<SparkFlex>(motor, motor.getClosedLoopController());
    }

    /** Creates a SparkMax that follows leader. Leader is NOT touched. */
    public static SparkMax InstantiateFollowerMax(MotorData data, SparkMax leader) {
        SparkMax motor = new SparkMax(data.getID(), SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = data.applyBaseTo(new SparkMaxConfig());
        config.follow(leader, false);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    /** Creates a SparkFlex that follows leader. Leader is NOT touched. */
    public static SparkFlex InstantiateFollowerFlex(MotorData data, SparkFlex leader) {
        SparkFlex motor = new SparkFlex(data.getID(), SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig config = data.applyBaseTo(new SparkFlexConfig());
        config.follow(leader, false);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    //#endregion Motor Instantiation
}