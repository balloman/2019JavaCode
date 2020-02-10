package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.Wheel.*;


public class DriveForDistanceCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final double distanceToDrive;

    public DriveForDistanceCommand(SwerveDriveSubsystem swerveDriveSubsystem, double distanceToDrive) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.distanceToDrive = distanceToDrive;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        if (distanceToDrive > swerveDriveSubsystem.getTalonDriveEncoder(FL)){
            swerveDriveSubsystem.Drive(0, 0.3, 0, true);
        }
    }

    @Override
    public boolean isFinished() {
        return swerveDriveSubsystem.getTalonDriveEncoder(FL) > distanceToDrive;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.Drive(0, 0, 0, false);
    }
}
