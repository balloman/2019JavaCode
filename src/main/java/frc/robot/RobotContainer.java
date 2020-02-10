/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CalibrateCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.ResetNavCommand;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final XboxController xboxController;
    private final NavXSubsystem navXSubsystem;


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // Configure the button bindings

        swerveDriveSubsystem = SwerveDriveSubsystem.getInstance();
        xboxController = new XboxController(0);
        navXSubsystem = NavXSubsystem.getInstance();
        swerveDriveSubsystem.setDefaultCommand(new DriveCommand(swerveDriveSubsystem, xboxController,
                true, navXSubsystem));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings()
    {
        JoystickButton combo = new JoystickButton(xboxController, XboxController.Button.kA.value);
        combo.whenReleased(new CalibrateCommand(swerveDriveSubsystem));
        new JoystickButton(xboxController, XboxController.Button.kBumperRight.value)
                .whenReleased(new ResetNavCommand(navXSubsystem));

        //DriveForDistance
        new JoystickButton(xboxController, XboxController.Button.kBumperLeft.value)
                .whenPressed(new DriveForDistanceCommand(swerveDriveSubsystem, 100));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return new DriveForDistanceCommand(swerveDriveSubsystem, 100);
    }
}
