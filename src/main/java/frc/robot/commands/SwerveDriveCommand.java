package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends Command {

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldRelativeSupplier;
    private final BooleanSupplier openLoopSupplier;

    public SwerveDriveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier openLoopSupplier) {

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.openLoopSupplier = openLoopSupplier;

        // Add the subsystem requirement using the instance from Constants
        addRequirements(Constants.swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        // Get the joystick inputs
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();
        double rotationSpeed = rotationSupplier.getAsDouble();
        boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();
        boolean isOpenLoop = openLoopSupplier.getAsBoolean();

        // Create the translation vector
        Translation2d translation = new Translation2d(xSpeed, ySpeed);

        // Command the swerve subsystem to drive using the instance from Constants
        Constants.swerveDriveSubsystem.drive(translation, rotationSpeed, fieldRelative, isOpenLoop);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the swerve drive when the command ends
        Constants.swerveDriveSubsystem.drive(new Translation2d(0, 0), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own
        return false;
    }
}
