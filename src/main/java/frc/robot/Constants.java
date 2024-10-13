// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorcontrollerPort=1;
    public static final int GyroID=21;
    public static final int kintakeRollerId = 31;
    public static final int kintakePivotid = 32;
    public static final boolean LEFT_DriveMOTOR_INVERTED=false;
    public static final boolean RIGHT_DriveMOTOR_INVERTED=true;
    public static final Double DEADBAND= 0.02;
    public static final boolean intakerollerinverted = false;
    public static final boolean intakepivotinverted = false;
    public static final int IntakeBoreID = 0;
    public static final double positionTolerance = 3;
    public static final double pivotkP= 0.2;
    public static final double pivotkI = 0.0;
    public static final double pivotkD = 0.0;
    public static final double INTAKE_OPEN_POSITION=210;
    public static final double INTAKE_CLOSED_POSITION=10;
    public static final double intakerollerspeed= 0.2;
    public static final int kShooterRId = 41;
    public static final int kShooterLId = 42;
    public static final boolean kShooterLInverted= false;
    public static final boolean kShooterRInverted= false;
    public static final double kshooterP= 0.2;
    public static final double kshooterI= 0.0;
    public static final double kshooterD= 0.0;
    public static final double kshooterpositionTolerance = 3;
    public static final double kshootervelocityTolerance = 1;
    public static final double shooterTargetRPM = 6000;
    public static final double intakerollerrealesespeed = 0.1;
    public static final double gearRatio = 8.46; //gear ratio of the drive //TO DO
    public static final double raduisOfWheelInMeters = 0.0762; //TO DO 
    public static final double circumferenceOfWheelInMeters = Math.PI*2*raduisOfWheelInMeters;
    public static final double conversionFactor = circumferenceOfWheelInMeters*gearRatio;
    public static final double trackwWidth= 0.53133; //in meters //TO DO
    public static final double kmaxspeedmps = 3;//TO DO
    public static final double kmaxaccmpssqr = 2;//TO DO
    public final static Intake intakeSubsystem = new Intake();
    public final static Shooter shooterSubsytem = new Shooter();
    public static final double SWERVEABSOLUTE_ENCODER_CONVERSION_FACTOR = 360;
    public static final double MaxModuleSpeed = 0;//TO_DO
    public static SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
	public static double kdrivebaseRadius = 0;//TO_DO
    public static double kTranslationP = 0;//TO_DO
    public static double kRotationP = 0;//TO_DO

    }

