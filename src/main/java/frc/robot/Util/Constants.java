// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 6; // radians per second
    public static final double kMagnitudeSlewRate = 4; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 8; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.8);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.8);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );
    
    public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kPigeonGyroID = 2;

    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 10;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.3;
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

  public static final class ElevatorConstants{
    public static final int kElevatorMasterCANId = 11;
    public static final int kElevatorSlaveCANId = 12;

    public static final double kElevatorP = 0.4;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;

    public static final double kElevatorSVolts = 0.5;
    public static final double kElevatorGVolts = 0.36;
    public static final double kElevatorVVoltSecperCm = 0.1;
    public static final double kMaxVelocityCmperSec = 70;
    public static final double kMaxAcceleratioCmperSecSquared = 100;

    public static final double kDirectionSlewRate = 1; //percent per second (1 = 100%)

    public static final double kElevatorSprocketPitchDiameter = 3.9; //cm
    public static final double kElevatorMotorReduction = 6; //6:1 gearbox
    public static final double kEncoderPositionFactor = (kElevatorSprocketPitchDiameter * Math.PI)
     / kElevatorMotorReduction; //cm
    public static final double kEncoderVelocityFactor = kEncoderPositionFactor
     / 60; //cm per sec
    public static final double kElevatorMaxHeightCm = 81;
    public static final double kElevatorMinHeightCm = 0;  
    public static final double kCarriageMassKg = 5;
  }

  public static final class ExtensorConstants{
    public static final int kExtensorCANId = 13;

    public static final double kExtensorP = 0.5;
    public static final double kExtensorI = 0;
    public static final double kExtensorD = 0;

    public static final double kExtensorkSVolts = 0.1;
    public static final double kExtensorVVoltSecperCm = 0.1;

    public static final double kMaxVelocityCmperSec = 20;
    public static final double kMaxAcceleratioCmperSecSquared = 30;

    public static final double kDirectionSlewRate = 1; //percent per second (1 = 100%)

    public static final double kExtensorSprocketPitchDiameter = 3.9; //cm
    public static final double kElevatorMotorReduction = 6; //6:1 gearbox
    public static final double kEncoderPositionFactor = (kExtensorSprocketPitchDiameter * Math.PI)
     / kElevatorMotorReduction; //cm
    public static final double kEncoderVelocityFactor = kEncoderPositionFactor
     / 60; //cm per sec

    public static final double kExtensorMaxExtension = 37;
    public static final double kExtensorMinExtension = -1;
    public static final double kCarriageMassKg = 0.1;
  }

  public static final class ClawConstants{
    public static final int kClawAngleCANId = 14;
    public static final int kClawAngleEncoderDIOPort = 1;
    public static final int kClawExpulsorCANId = 15;

    public static final int kWristMotorReduction = 80 * (28/15);
    //public static final double kWristEncoderPositionFactor = (2 * Math.PI) / (kWristEncoderCPR *  kWristMotorReduction);
    public static final double kWristEncoderPositionFactor = (2 * Math.PI);


    public static final double kClawAngleP = 10;
    public static final double kClawAngleI = 0;
    public static final double kClawAngleD = 0;

    public static final double kClawSVolts = 0.3;
    public static final double kClawGVolts = 0.4;
    public static final double kClawVVoltSecPerRad = 1.2;

    public static final double kClawMaxVelocityRadPerSec = Math.PI / 1.4;
    public static final double kClawMaxAccelerationRadPerSecSquared = Math.PI/2;

    public static final double kWristMaxRotationRad = 2.5;
    public static final double kWristMinRotationRad = -0.37;  

    public static final double kExpulsorEncoderVelocityFactor = 20 / 60;


  }

  public static final class VisionConstants{
    public static final Transform3d kTransformRobotToCam = new Transform3d(
      new Translation3d(-0.32, -0.095, 1.33),
      new Rotation3d(
              0, 0,
              0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
// from center.
    public static final String kCameraName = "6605Camera";

    public static final double kXControllerP = 0.13;
    public static final double kXControllerD = 0;
    public static final double kYControllerP = 0.21;
    public static final double kYControllerD = 0;
    public static final double kOmegaControllerP = 0.22;
    public static final double kOmegaControllerD = 0;

    public static final double kMaxXVelocity = 2;
    public static final double kMaxXAcceleration = 3;

    public static final double kMaxYVelocity = 2;
    public static final double kMaxYAcceleration = 3;

    public static final double kMaxOmegaVelocity = 1;
    public static final double kMaxOmegaAcceleration = 1.5;
  }

  public static final class LEDConstants{
    public static final int kLEDPort = 0;
  }

  public static final class OIConstants {
    public static final String kDriverControllerType = "PS4";
    public static final int kDriverControllerPort = 0;
    public static final int kCommandsControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static boolean isManual = false;
    public static boolean isCargoCube = true;

    public static final int kLogitechLeftYAxis = 1;
    public static final int kLogitechLeftXAxis = 0;
    public static final int kLogitechRightYAxis = 3;
    public static final int kLogitechRightXAxis = 2;

    public static final int kLogitechDownButton= 2;
    public static final int kLogitechRightButton= 3;
    public static final int kLogitechLeftButton= 1;
    public static final int kLogitechUpButton= 4;
    public static final int kLogitechR1= 6;
    public static final int kLogitechL1= 5;
    public static final int kLogitechR2= 7;
    public static final int kLogitechL2= 8;

  }

  public static final class AutoConstants {
    public static final PathConstraints kAutoConstraints = new PathConstraints(1, 2);

    public static final HashMap<String, Command> kEventMap = new HashMap<>();

    //Balance Constants
    public static final double kBalanceGoalDegrees = 0;
    public static final double kBalanceAngleTresholdDegrees = 5;
    public static final double kBalancedDrivekP = 0.09;
    public static final double kBalancedDrivekD = 0.01;
    public static final TrapezoidProfile.Constraints kBalanceConstraints = new TrapezoidProfile.Constraints(
      1.0,
     0.6);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
