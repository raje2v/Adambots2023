/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import org.opencv.core.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {

    public static final int RING_LIGHT_PORT = 4;

    public static ShuffleboardTab debugTab;

    public static final class DriveConstants {
        // Tank Drive Constants
        public static final double kNormalSpeedModifier = 1;
        public static final double kLowSpeedModifier = 0.5;
        // Acceleration ramping constant for drive train
        public static final double kNeutralToFull = 0.4;
        public static final int kDrivePIDSlot = 0;
        public static final double kEncoderTickerPerInch = 3500;

        // Swerve Drive Constants
        public static final int kFrontLeftDriveMotorPort = 18;
        public static final int kRearLeftDriveMotorPort = 16;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kRearRightDriveMotorPort = 14;

        public static final int kFrontLeftTurningMotorPort = 17;
        public static final int kRearLeftTurningMotorPort = 15;
        public static final int kFrontRightTurningMotorPort = 11;
        public static final int kRearRightTurningMotorPort = 13;

        public static final int kRearLeftEncoderPort = 2;
        public static final int kRearRightEncoderPort = 3;
        public static final int kFrontRightEncoderPort = 5;
        public static final int kFrontLeftEncoderPort = 4;

        // public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
        // public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
        // public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
        // public static final int[] kRearRightTurningEncoderPorts = new int[] {6, 7};

        public static final boolean kFrontLeftTurningEncoderReversed = true; // false
        public static final boolean kRearLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = true; // false
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final int[] kFrontLeftDriveEncoderPorts = new int[] { 8, 9 };
        public static final int[] kRearLeftDriveEncoderPorts = new int[] { 10, 11 };
        public static final int[] kFrontRightDriveEncoderPorts = new int[] { 12, 13 };
        public static final int[] kRearRightDriveEncoderPorts = new int[] { 14, 15 };

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = true;

        // In Meters
        public static final double kTrackWidth = 0.61; // 0.5
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.61; // 0.7

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for
        // your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = 3;
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static double kPModuleTurningController = -0.9;
        public static final double kIModuleTurningController = 0;
        public static double kDModuleTurningController = -0.02;
        public static final String kPTurningKey = "kPTurningKey";
        public static final String kDTurningKey = "kDTurningKey";

        public static final double kPModuleDriveController = 1;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public final static I2C.Port I2C_PORT = I2C.Port.kOnboard;
        public static final int LIDAR_DIO = 5; //Not used
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.05;
        public static final double kPYController = 0.05;
        public static final double kPThetaController = -0.2;
    	public static final double GYRO_TOLERANCE = 0.5; //degrees tolerance for measurement

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionConstants {
        public static final int kBackCamNumber = 1;
        public static final int kFrontCamNumber = 0;
        public static final int kCamExposure = 5;
        public static final int kFrameWidth = 320;
        public static final int kFrameHeight = 240;
        public static final int kHorizontalFOVDegrees = 60;
        public static final double kHorizontalDegreesPerPixel = (double) kHorizontalFOVDegrees / kFrameWidth;
        public static final int kImageHorizontalMidPoint = kFrameWidth / 2;
        public static final int kDriverStationFramesPerSec = 6;
        public static final int kProcessingFramesPerSec = 30; // DON'T CHANGE

        public static final double kCameraFieldOfView = 68.5;

    }
}
