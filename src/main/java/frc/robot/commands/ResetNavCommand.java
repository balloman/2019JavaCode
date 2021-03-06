package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavXSubsystem;


public class ResetNavCommand extends CommandBase {
    private final NavXSubsystem navXSubsystem;

    public ResetNavCommand(NavXSubsystem navXSubsystem) {
        this.navXSubsystem = navXSubsystem;
        addRequirements(navXSubsystem);
    }

    @Override
    public void initialize() {
        navXSubsystem.reset();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
