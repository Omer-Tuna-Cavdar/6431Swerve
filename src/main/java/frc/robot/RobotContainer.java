package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.SwerveDriveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    // Subsystems
    private final SwerveDriveSubsystem swerveDriveSubsystem = Constants.swerveDriveSubsystem;

    // Controllers and Buttons
    private final PS5Controller driverController = new PS5Controller(Constants.kDriverControllerPort);
    private final JoystickButton l1Button = new JoystickButton(driverController, PS5Controller.Button.kL1.value);
    private final JoystickButton r2Button = new JoystickButton(driverController, PS5Controller.Button.kR2.value);

    // Commands
    ToggleIntakeCommand toggleIntakeCommand = new ToggleIntakeCommand(Constants.intakeSubsystem);
    ShootCommand shootCommand = new ShootCommand(Constants.shooterSubsytem, Constants.intakeSubsystem, Constants.shooterTargetRPM);

    // Auto
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Deadband values for joystick axes
    private static final double LEFT_Y_DEADBAND = 0.05;
    private static final double LEFT_X_DEADBAND = 0.05;
    private static final double RIGHT_X_DEADBAND = 0.05;

    public RobotContainer() {
        SwerveDriveDriveCommand closedAbsoluteDriveAdv = new SwerveDriveDriveCommand(Constants.swerveDriveSubsystem,
                                                                              () -> MathUtil.applyDeadband(-driverController.getLeftY(), LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driverController.getLeftX(), LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverController.getRightX(), RIGHT_X_DEADBAND)
                                                                                        );


        // Set default drive command depending on simulation or real robot
        swerveDriveSubsystem.setDefaultCommand(
          closedAbsoluteDriveAdv
        );

        // Configure button bindings
        configureButtonBindings();

        // Register auto commands to the chooser
        autoChooser.setDefaultOption("2NoteAuto", new PathPlannerAuto("2NoteAuto"));
        NamedCommands.registerCommand("openintake", toggleIntakeCommand);
        NamedCommands.registerCommand("shootCommand", shootCommand);

        // Display the autonomous chooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {
        // Configure button bindings for the controller
        l1Button.onTrue(toggleIntakeCommand);
        r2Button.onTrue(shootCommand);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
