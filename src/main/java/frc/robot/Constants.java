package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

  public static final class KinematicsConstants {
    public static final double KINEMATICS_CHASSIS_WIDTH = 0.635; // Distance between right and left wheels
    public static final double KINEMATICS_CHASSIS_LENGTH = 0.635; // Distance between front and back wheels
    public static final SwerveDriveKinematics KINEMATICS_DRIVE_CHASSIS = new SwerveDriveKinematics(
      new Translation2d(+KINEMATICS_CHASSIS_WIDTH / 2, +KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(+KINEMATICS_CHASSIS_WIDTH / 2, -KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(-KINEMATICS_CHASSIS_WIDTH / 2, +KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(-KINEMATICS_CHASSIS_WIDTH / 2, -KINEMATICS_CHASSIS_LENGTH / 2)
    );

    public static final double RADIUS_DRIVE_CHASSIS = Math.sqrt((KINEMATICS_CHASSIS_LENGTH/2)*(KINEMATICS_CHASSIS_LENGTH/2) + (KINEMATICS_CHASSIS_WIDTH/2)*(KINEMATICS_CHASSIS_WIDTH/2));
  }

  public static final class ModuleConstants {

    //Robot Geometry
    public static final double MODULE_WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double MODULE_DRIVE_GEAR_RATIO = 6.75 / 1.0; // Drive ratio of 8.14 : 1
    public static final double MODULE_TURN_GEAR_RATIO = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1 //????
    public static final double MODULE_DRIVE_ROTATIONS_TO_METERS = ((MODULE_WHEEL_DIAMETER * Math.PI) / MODULE_DRIVE_GEAR_RATIO);/// 0.77;
    public static final double MODULE_TURN_ROTATIONS_TO_RADIANS = MODULE_TURN_GEAR_RATIO * 2 * Math.PI;
    public static final double MODULE_DRIVE_RPM_TO_MPS = MODULE_DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double TurningEncoderRPM2RadPerSec = MODULE_TURN_ROTATIONS_TO_RADIANS / 60.0;
  }

  public static class SwerveSubsystemConstants {


    // DRIVE Motor Ports
    // public static final int ID_FRONT_LEFT_DRIVE = 51;//53
    // public static final int ID_BACK_LEFT_DRIVE = 54; //52
    // public static final int ID_FRONT_RIGHT_DRIVE = 52;//54
    // public static final int ID_BACK_RIGHT_DRIVE = 53; //51

    // // TURNING Motor Ports
    // public static final int ID_FRONT_LEFT_TURN = 31;//33
    // public static final int ID_BACK_LEFT_TURN = 34;//32
    // public static final int ID_FRONT_RIGHT_TURN = 32;//34
    // public static final int ID_BACK_RIGHT_TURN = 33; //31

    public static final int ID_FRONT_LEFT_DRIVE = 53;
    public static final int ID_BACK_LEFT_DRIVE = 52;
    public static final int ID_FRONT_RIGHT_DRIVE = 54;
    public static final int ID_BACK_RIGHT_DRIVE = 51;

    // TURNING Motor Ports
    public static final int ID_FRONT_LEFT_TURN = 33;
    public static final int ID_BACK_LEFT_TURN = 32;
    public static final int ID_FRONT_RIGHT_TURN = 34;
    public static final int ID_BACK_RIGHT_TURN = 31;

    // CANCoder Ids
    public static final int ID_FRONT_LEFT_ENCODER_ABSOLUTE = 2; //0
    public static final int ID_BACK_LEFT_ENCODER_ABSOLUTE = 1;//3
    public static final int ID_FRONT_RIGHT_ENCODER_ABSOLUTE = 3;//1
    public static final int ID_BACK_RIGHT_ENCODER_ABSOLUTE = 0;//2

    // Invert booleans | We use MK4i modules so the turning motors are inverted
    public static final boolean REVERSED_ENCODER_TURN = true;
    public static final boolean REVERSED_ENCODER_DRIVE = false;
    public static final boolean REVERSED_ENCODER_ABSOLUTE = false;
    public static final boolean REVERSED_GYRO = false;

    // Invert Specific Motors
    public static final boolean REVERSED_FRONT_LEFT_MOTOR_DRIVE = true;//f
    public static final boolean REVERSED_FRONT_RIGHT_MOTOR_DRIVE = false;//t
    public static final boolean REVERSED_BACK_LEFT_MOTOR_DRIVE = true;//f
    public static final boolean REVERSED_BACK_RIGHT_MOTOR_DRIVE = false;//t

    // Turning encoder offsets
    public static final double OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE = 1.88506; //-1.88758;
    public static final double OFFSET_BACK_LEFT_ENCODER_ABSOLUTE  = -2.91636; //-2.5;
    public static final double OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE= -2.5; //-2.91636;
    public static final double OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE = -1.88758;//1.88506;

    // Robot drive speeds
    public static final double LIMIT_HARD_SPEED_DRIVE = 3.6; // hard limit for speed of chassis
    public static final double LIMIT_SOFT_SPEED_DRIVE = 1.4; //2.4 soft limit for speed of chassis

    // Robot turning speeds
    public static final double LIMIT_SOFT_SPEED_TURN = 1 * 2*Math.PI; // soft limit for module rotation

    // Robot acceleration
    public static final double LIMIT_SOFT_ACCELERATION_SPEED = 0.6; //1 soft limit for acceleration (M/S^2)
    public static final double LIMIT_SOFT_ACCELERATION_TURN = 0.6;  //1 soft limit for acceleration (M/S^2)
  }

  public static class ArmConstants{

    public static int ID_LEFT_ARM = 11;
    public static int ID_RIGHT_ARM = 12;
    public static double LIMIT_AUTOSPEED_ROTATE = 360;

  }

  public static class AutonomousConstants{

    // Shop zone 332cm x 173cm
    public static double LIMIT_AUTOSPEED_DRIVE = 3.5;
    public static double LIMIT_AUTOSPEED_ROTATE = 360;
  }


}