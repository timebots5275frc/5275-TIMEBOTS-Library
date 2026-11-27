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

@SuppressWarnings("removal")
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

    /**
     * Creates and fully configures a SparkMax for POSITION control in custom units (for example
     * swerve steering in wheel degrees). Everything InstantiateMax does (ID, current limits, idle
     * mode, gains from the MotorData's PID) plus the position-specific settings, applied in a
     * single configure call.
     *
     * Usage:  var m = Helpers.InstantiateMaxPosition(steerData, 360.0 * STEER_GEAR_RATIO, true, -180, 180);
     *         m.getMotorController().setSetpoint(90, SparkMax.ControlType.kPosition);
     *
     * @param data                     ID, current limits, and PID gains (gains must be POSITION gains)
     * @param positionConversionFactor units per motor rotation (e.g. wheel degrees per motor rotation)
     * @param inverted                 whether to invert the motor (also flips the built-in encoder's sign)
     * @param wrapMin                  lowest value of the wrapping range, in the converted units (e.g. -180)
     * @param wrapMax                  highest value of the wrapping range, in the converted units (e.g. 180)
     */
    public static MotorAndController<SparkMax> InstantiateMaxPosition(MotorData data, double positionConversionFactor, boolean inverted, double wrapMin, double wrapMax) {
        SparkMax motor = new SparkMax(data.getID(), SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = data.applyTo(new SparkMaxConfig());
        config.inverted(inverted);

        // Make the built-in encoder report the converted units instead of motor rotations
        config.encoder.positionConversionFactor(positionConversionFactor);

        // Feedback is the primary (built-in) encoder, which is the default for a brushless motor
        config.closedLoop
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)             // take the shortest path around the circle
            .positionWrappingInputRange(wrapMin, wrapMax);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return new MotorAndController<SparkMax>(motor, motor.getClosedLoopController());
    }

    /**
     * Creates and fully configures a SparkMax for POSITION control in custom units (for example
     * swerve steering in wheel degrees). Everything InstantiateMax does (ID, current limits, idle
     * mode, gains from the MotorData's PID) plus the position-specific settings, applied in a
     * single configure call.
     *
     * Usage:  var m = Helpers.InstantiateMaxPosition(steerData, 360.0 * STEER_GEAR_RATIO, true, -180, 180);
     *         m.getMotorController().setSetpoint(90, SparkMax.ControlType.kPosition);
     *
     * @param data                     ID, current limits, and PID gains (gains must be POSITION gains)
     * @param positionConversionFactor units per motor rotation (e.g. wheel degrees per motor rotation)
     * @param inverted                 whether to invert the motor (also flips the built-in encoder's sign)
     * @param wrapMin                  lowest value of the wrapping range, in the converted units (e.g. -180)
     * @param wrapMax                  highest value of the wrapping range, in the converted units (e.g. 180)
     */
    public static MotorAndController<SparkFlex> InstantiateFlexPosition(MotorData data, double positionConversionFactor, boolean inverted, double wrapMin, double wrapMax) {
        SparkFlex motor = new SparkFlex(data.getID(), SparkLowLevel.MotorType.kBrushless);

        SparkFlexConfig config = data.applyTo(new SparkFlexConfig());
        config.inverted(inverted);

        // Make the built-in encoder report the converted units instead of motor rotations
        config.encoder.positionConversionFactor(positionConversionFactor);

        // Feedback is the primary (built-in) encoder, which is the default for a brushless motor
        config.closedLoop
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)             // take the shortest path around the circle
            .positionWrappingInputRange(wrapMin, wrapMax);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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