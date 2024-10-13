package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import java.util.function.BooleanSupplier;

public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveDrive m_swerve;

    public SwerveDriveSubsystem(File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        try {
            m_swerve = new SwerveParser(directory).createSwerveDrive(Constants.kmaxspeedmps);

            SwerveModule[] modules = m_swerve.getModules();
            for (SwerveModule module : modules) {
                module.getAngleMotor().configureIntegratedEncoder(Constants.SWERVEABSOLUTE_ENCODER_CONVERSION_FACTOR);
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Create the HolonomicPathFollowerConfig
        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                        new PIDConstants(Constants.kTranslationP, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.kRotationP, 0.0, 0.0), // Rotation PID constants
                        Constants.MaxModule, Constants.kdrivebaseRadius, new ReplanningConfig()
        );

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                speeds -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                config, // Pass the HolonomicPathFollowerConfig
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    // Add the resetPose method
    public void resetPose(Pose2d pose) {
        m_swerve.resetOdometry(pose);
    }

    // Add the getRobotRelativeSpeeds method
    public ChassisSpeeds getRobotRelativeSpeeds() {
        // Get field-relative chassis speeds
        ChassisSpeeds fieldRelativeSpeeds = m_swerve.getFieldVelocity();

        // Convert to robot-relative speeds using the current yaw angle
        Rotation2d robotYaw = getYaw();
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond,
            fieldRelativeSpeeds.omegaRadiansPerSecond,
            robotYaw
        );
    }

    // Add the driveRobotRelative method
    public void driveRobotRelative(ChassisSpeeds speeds) {
        m_swerve.drive(speeds);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        m_swerve.drive(translation.rotateBy(Rotation2d.fromDegrees(-90)), rotation, fieldRelative, isOpenLoop);
    }

    public ChassisSpeeds getTargetSpeeds(double x, double y, Rotation2d rotation) {
        return m_swerve.swerveController.getTargetSpeeds(x, y, rotation.getRadians(),
                getYaw().getRadians(), m_swerve.getMaximumVelocity());
    }

    @Override
    public void periodic() {
    }

    public void setBrakeMode(boolean brake) {
        m_swerve.setMotorIdleMode(brake);
    }

    public ChassisSpeeds getFieldVelocity() {
        return m_swerve.getFieldVelocity();
    }

    public SwerveDriveKinematics getKinematics() {
        return m_swerve.kinematics;
    }

    public void zeroGyro() {
        m_swerve.zeroGyro();
    }

    public void setGyro(double degrees) {
        m_swerve.setGyro(new Rotation3d(0, 0, degrees));
    }

    public Rotation2d getYaw() {
        return m_swerve.getYaw();
    }

    public Rotation2d getRoll() {
        return m_swerve.getRoll();
    }

    public Rotation2d getPitch() {
        return m_swerve.getPitch();
    }

    public Pose2d getPose() {
        return m_swerve.getPose();
    }

    public void resetPoseEstimator(Pose2d pose) {
        m_swerve.resetOdometry(pose);
    }

    public void postTrajectory(Trajectory trajectory) {
        m_swerve.postTrajectory(trajectory);
    }

    public void updatePoseEstimator() {
        m_swerve.updateOdometry();
    }
}
