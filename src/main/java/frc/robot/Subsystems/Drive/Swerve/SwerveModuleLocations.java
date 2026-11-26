package frc.robot.Subsystems.Drive.Swerve;

/**
 * Plain data holder for the location of each swerve wheel relative to the
 * center of the robot, in METERS.
 *
 * <p>Coordinates follow the WPILib convention (right-hand rule):
 * <ul>
 *   <li>+X points forward (index finger)</li>
 *   <li>+Y points left (middle finger)</li>
 *   <li>+Z points up (thumb)</li>
 * </ul>
 * Module names use the abbreviations LF = left front, RF = right front,
 * RR = right rear, LR = left rear.
 */
public class SwerveModuleLocations {

    // Left front wheel
    public final double LEFT_FRONT_WHEEL_X;
    public final double LEFT_FRONT_WHEEL_Y;

    // Right front wheel
    public final double RIGHT_FRONT_WHEEL_X;
    public final double RIGHT_FRONT_WHEEL_Y;

    // Right rear wheel
    public final double RIGHT_REAR_WHEEL_X;
    public final double RIGHT_REAR_WHEEL_Y;

    // Left rear wheel
    public final double LEFT_REAR_WHEEL_X;
    public final double LEFT_REAR_WHEEL_Y;

    /**
     * Arguments are an (X, Y) pair per wheel, in the order LF, RF, RR, LR
     * (the same order the kinematics and modules use elsewhere). All values in meters.
     */
    public SwerveModuleLocations(double LFWX, double LFWY, double RFWX, double RFWY, double RRWX, double RRWY, double LRWX, double LRWY)
    {
        LEFT_FRONT_WHEEL_X = LFWX;
        LEFT_FRONT_WHEEL_Y = LFWY;
        RIGHT_FRONT_WHEEL_X = RFWX;
        RIGHT_FRONT_WHEEL_Y = RFWY;
        RIGHT_REAR_WHEEL_X = RRWX;
        RIGHT_REAR_WHEEL_Y = RRWY;
        LEFT_REAR_WHEEL_X = LRWX;
        LEFT_REAR_WHEEL_Y = LRWY;
    }
}