// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final class CanConstants {

      public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
      public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
      public static final int FRONT_LEFT_MODULE_STEER_CANCODER = 6;
      public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 254.8;//

      public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
      public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
      public static final int FRONT_RIGHT_MODULE_STEER_CANCODER = 9;
      public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 285.7;

      public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
      public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
      public static final int BACK_LEFT_MODULE_STEER_CANCODER = 12;
      public static final double BACK_LEFT_MODULE_STEER_OFFSET = 205.8;//

      public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
      public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14;
      public static final int BACK_RIGHT_MODULE_STEER_CANCODER = 15;
      public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 62.8;

    }

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kBackLeftDrivingCanId = 10;
    public static final int kBackRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kBackLeftTurningCanId = 11;
    public static final int kBAckRightTurningCanId = 14;

    public static final int kFrontLeftCancoderId = 6;
    public static final int kFrontRightCancoderId = 9;
    public static final int kBackLeftCancoderId = 12;
    public static final int kBackRightCancoderId = 15;

    public static final double kFrontLeftCancoderOffset = 254.8;
    public static final double kBackLeftCancoderOffset = 285.7;
    public static final double kFrontRightCancoderOffset = 205.8;
    public static final double kBackRightCancoderOffset = 62.8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;
    public static final boolean kCancoderInverted = true;


    
  
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14.122807

    public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

    public static final double kDriveMetersPerEncRev =

        (kWheelDiameterMeters * Math.PI) / mk4iL1DriveGearRatio;// 0.039198257811106

    public static double kEncoderRevsPerMeter = 1 / kDriveMetersPerEncRev;// 25.511337897182322

    public static final double kTurningDegreesPerEncRev = 360 / mk4iL1TurnGearRatio;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
 
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = mk4iL1DriveGearRatio;

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = kTurningDegreesPerEncRev;//(2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
