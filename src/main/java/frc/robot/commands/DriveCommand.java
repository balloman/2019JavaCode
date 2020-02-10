package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import com.kauailabs.navx.frc.*;
import frc.robot.utils.SwerveMath;


public class DriveCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final XboxController driverController;
    private final NavXSubsystem navXSubsystem;
    private final boolean fieldCentric;

    public DriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, XboxController driverController,
                        boolean fieldCentric, NavXSubsystem navXSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.driverController = driverController;
        this.navXSubsystem = navXSubsystem;
        this.fieldCentric = fieldCentric;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (fieldCentric){
            double[] rotated = SwerveMath.rotateHeading(driverController.getX(GenericHID.Hand.kLeft),
                    driverController.getY(GenericHID.Hand.kLeft), navXSubsystem.getHeading());

            //Actually drive here:
            swerveDriveSubsystem.Drive(rotated[0], rotated[1], driverController.getTriggerAxis(GenericHID.Hand.kRight)
            + -driverController.getTriggerAxis(GenericHID.Hand.kLeft), false);
        }else{
            swerveDriveSubsystem.Drive(driverController.getX(GenericHID.Hand.kLeft),
                    driverController.getY(GenericHID.Hand.kLeft),
                    driverController.getTriggerAxis(GenericHID.Hand.kRight) +
                            -driverController.getTriggerAxis(GenericHID.Hand.kLeft), false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
