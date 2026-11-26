package frc.robot.Commands.Drive.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Swerve.SwerveDriveSubsystem;
import frc.robot.Subsystems.Input.Input;

public class SwerveDriveCommand extends Command {

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Input input;

    private double xSpeed;
    private double ySpeed;
    private double zRotation;

    public SwerveDriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, Input input) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.input = input;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        xSpeed = input.getSquaredScaledDriveInput().x;
        ySpeed = input.getSquaredScaledDriveInput().y;
        zRotation = input.getSquaredScaledDriveTurn();

        swerveDriveSubsystem.drive(xSpeed, ySpeed, zRotation, input.getFieldRelative());
    }
    
}
