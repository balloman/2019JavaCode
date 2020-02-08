package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import com.kauailabs.navx.frc.*;
import frc.robot.utils.SwerveMath;

/**
 * Handles TeleopDriving
 */
public class DriveCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final XboxController driverController;
    private final NavXSubsystem navXSubsystem;
    private final boolean fieldCentric;

    /**
     * Instantiate the command, mainly for Teleop Use, but You can probably override this
     * @param swerveDriveSubsystem The subsystem for the swerve drive
     * @param driverController The controller that will be used to drive the bot
     * @param fieldCentric Whether to drive field centric or not
     * @param navXSubsystem The subsystem for the navx
     */
    public DriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, XboxController driverController,
                        boolean fieldCentric, NavXSubsystem navXSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.driverController = driverController;
        this.navXSubsystem = navXSubsystem;
        this.fieldCentric = fieldCentric;
        addRequirements(swerveDriveSubsystem);
    }

    /**
     * Runs everytime teleop runs as long as no other drive commands are being issued
     */
    @Override
    public void execute() {
        if (fieldCentric){
            //Rotate with navx angle for field centric
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

    //What finishes this command, since return false, run continuously
    @Override
    public boolean isFinished() {
        return false;
    }

    //What to do if the command is interrupted
    @Override
    public void end(boolean interrupted) {

    }
}
