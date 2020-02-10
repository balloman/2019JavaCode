package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class CalibrateCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    public CalibrateCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.calibrate();
        swerveDriveSubsystem.setOffsets();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
