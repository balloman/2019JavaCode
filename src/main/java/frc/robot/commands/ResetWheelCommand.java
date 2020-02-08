package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class ResetWheelCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    public ResetWheelCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        for (Constants.Wheel wheel :
                swerveDriveSubsystem.getRefnums().keySet()) {
            swerveDriveSubsystem.setRotationMotors(wheel, swerveDriveSubsystem.getOffsets().get(wheel));
        }
    }

    @Override
    public boolean isFinished() {
        for (Constants.Wheel wheel :
                swerveDriveSubsystem.getRefnums().keySet()) {
            if (swerveDriveSubsystem.getRefnums().get(wheel).getMotorOutputPercent() > 0.1){
                return false;
            }
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.resetEncoders();
    }
}
