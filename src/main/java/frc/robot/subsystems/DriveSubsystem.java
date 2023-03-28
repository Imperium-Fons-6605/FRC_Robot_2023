// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
//Hello
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.invoke.VolatileCallSite;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import frc.robot.RobotContainer;
import frc.robot.Util.SwerveUtils;
import frc.robot.Util.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft;
  private final MAXSwerveModule m_frontRight;
  private final MAXSwerveModule m_rearLeft;
  private final MAXSwerveModule m_rearRight;

  private final MAXSwerveModule[] SwerveModules;


  

  // The gyro sensor
  private final Pigeon2 pigeonGyro = new Pigeon2(DriveConstants.kPigeonGyroID);
  private final SwerveDrivePoseEstimator m_poseEstimator;
  boolean turning_first_flag = false;
  boolean turning_second_flag = false;
  boolean turning_third_flag = false;


  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private double VoltagePot;
  private boolean isVelocityReduction = false;
  private LinearFilter voltageFilter = LinearFilter.singlePoleIIR(2, 0.02);
  double tmpVoltagePot;

  // Odometry class for tracking robot pose

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

    m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

    m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);
    
      m_rearRight = new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);
    
    SwerveModules = new MAXSwerveModule[]{m_frontLeft,m_frontRight,m_rearLeft,m_rearRight};
    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(pigeonGyro.getYaw()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      new Pose2d());

    SmartDashboard.putData("Zero heading", new InstantCommand(() -> zeroHeading()));


    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPosePitch = 0;
    config.MountPoseRoll = 0;
    config.MountPoseYaw = 0;
    pigeonGyro.configAllSettings(config);

    zeroHeading();
    resetEncoders();
  }

  @Override
  public void periodic() {
    VoltagePot = ((m_powerDistribution.getVoltage() - 8)/(11.5-8));
    SmartDashboard.putNumber("Position X",getPose().getX());
    SmartDashboard.putNumber("Position Y",getPose().getY());
    SmartDashboard.putNumber("Position Rotation",getPose().getRotation().getRadians());
    SmartDashboard.putNumber("Gyro Yaw",getHeading());
    SmartDashboard.putNumber("Gyro Pitch", pigeonGyro.getPitch());
    // Update the odometry in the periodic block
    updateOdometry();

  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_poseEstimator.update(
      Rotation2d.fromDegrees(pigeonGyro.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
    /* 
    Optional<EstimatedRobotPose> result =
            pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        m_poseEstimator.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    } else {
        // move it way off the screen to make it disappear
    }
    */
}

  /**
   * Method for reducing velocity depending on current voltage power, 
   * with a minimum power of 20%
   * @return Max percent power (1 = 100%)
   */
  public double getVoltagePot(){
    double currentVoltagePot;
    
    //Filters voltage current to reduct noisy signal due voltage peaks
    tmpVoltagePot = voltageFilter.calculate(VoltagePot);
    if (isVelocityReduction == true){
      tmpVoltagePot *= 0.15; 
    }

    //Clamping values between 1 and 0.2
    if (tmpVoltagePot > 1){
      currentVoltagePot =  1;
    } else if (tmpVoltagePot < 0.2) {
      currentVoltagePot =  0.2;
    } else {
      currentVoltagePot = tmpVoltagePot;
    }
    /* 
    if ((VoltagePot < 7.5) & !turning_third_flag) {
      for (MAXSwerveModule module: SwerveModules){
        module.getTurningPIDController().setP(0.15);
        turning_third_flag = true;
      }
    }

    if ((VoltagePot < 8) & !turning_second_flag) {
      for (MAXSwerveModule module: SwerveModules){
        module.getTurningPIDController().setP(0.2);
        turning_second_flag = true;
      }
    }

    if ((VoltagePot < 9) & !turning_first_flag) {
      for (MAXSwerveModule module: SwerveModules){
        module.getTurningPIDController().setP(0.25);
        turning_first_flag = true;
      }
    }
    */

    return currentVoltagePot;
  }

  /**
   * Reducts max chasis power in order to increase driver accuracy
   * @param reductVelocity If should reduct velocity
   */
  public void reductVelocity(boolean reductVelocity){
    isVelocityReduction = reductVelocity;
  }

  public Pigeon2 getPigeonGyro() {
      return pigeonGyro;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(pigeonGyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(pigeonGyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeonGyro.setYaw(0);
    pigeonGyro.setAccumZAngle(0);

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(pigeonGyro.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double[] degreesPerSecond = new double[] {0.0,0.0,0.0};
    pigeonGyro.getRawGyro(degreesPerSecond);
    return  degreesPerSecond[2]* (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
