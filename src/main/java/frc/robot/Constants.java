// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

  public static class PIDCoefficients {
    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0.005;
    public static double kIz = 0;
    public static double kFF = 0;
    public static double kMaxOutput = 1;
    public static double kMinOutput = -1;
  }
  public static class SwerveConstants{
    public static final class ModuleConstants {
        // public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        // public static final double kDriveMotorGearRatio = 1 / 5.8462;
        // public static final double kTurningMotorGearRatio = 1 / 18.0;
        // public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        // public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        // public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        // public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        // public static final double kPTurning = 0.1;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75; // 1 / 8.14 ; // old value 1 / 5.8462
        public static final double kTurningMotorGearRatio = 1 / 12.8; // old value 1 / 18.0
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.1;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.635;
        public static final double kWheelBase = 0.635;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // + olacak
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // - olacak
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // + olacak
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // - olacak
                
        //TODO: ports should be changed.
        public static final int kFrontLeftDriveMotorPort = 51;
        public static final int kBackLeftDriveMotorPort = 54;
        public static final int kFrontRightDriveMotorPort = 52;
        public static final int kBackRightDriveMotorPort = 53;

        public static final int kFrontLeftTurningMotorPort = 31;
        public static final int kBackLeftTurningMotorPort = 34;
        public static final int kFrontRightTurningMotorPort = 32;
        public static final int kBackRightTurningMotorPort = 33;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 68 * Math.PI / 180;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 30 * Math.PI / 180;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -170 * Math.PI / 180;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 104 * Math.PI / 180;

        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.68 * 2 * Math.PI - Math.PI; //0.98 * 2 * Math.PI;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.613 * 2 * Math.PI - Math.PI;        // 1.04719      //1.04719
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.0448 * 2 * Math.PI - Math.PI; //(0.0141+.25) * 2 * Math.PI;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  0.795 * 2 * Math.PI - Math.PI; //0.2577 * 2 * Math.PI;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.88758;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -2.5;        
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -2.91636;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.88506;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 15;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond / 1;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 12;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 10;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI;
        public static final double kPXController = 0.05;
        public static final double kPYController = 0.05;
        public static final double kPThetaController = 0.5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.03;
    }

    public static final class ArmConstants {
        public static final int kLeftArmMotorId = 11;
        public static final int kRightArmMotorId = 12;
    }
  }
}
