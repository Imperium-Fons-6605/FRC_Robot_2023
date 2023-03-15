package frc.robot.Commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrackApriltag extends CommandBase{
    /* 
    private final TrapezoidProfile.Constraints XConstraints = new Constraints(
        VisionConstants.kMaxXVelocity, 
        VisionConstants.kMaxXAcceleration);
    private final TrapezoidProfile.Constraints YConstraints = new Constraints(
        VisionConstants.kMaxYVelocity, 
        VisionConstants.kMaxYAcceleration);
    private final TrapezoidProfile.Constraints OmegaConstraints = new Constraints(
        VisionConstants.kMaxOmegaVelocity, 
        VisionConstants.kMaxOmegaAcceleration);

    private static final Transform3d tagToUpperGoal = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(15),Units.inchesToMeters(10),Units.inchesToMeters(0)), 
            new Rotation3d(0, 0, Math.PI)
        );
    private static final Transform3d tagToMidGoal = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(15),Units.inchesToMeters(0),Units.inchesToMeters(0)), 
            new Rotation3d(0, 0, Math.PI)
        );
    private static final Transform3d tagToDownGoal = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(15),Units.inchesToMeters(-10),Units.inchesToMeters(0)), 
            new Rotation3d(0, 0, Math.PI)
        );
    private static final Transform3d[] goalPositions = {tagToDownGoal,tagToMidGoal,tagToUpperGoal};

    private final PhotonCamera m_camera;
    private final DriveSubsystem m_driveSubsystem;
    private final Supplier<Pose2d> m_robotPoserProvider;
    private int tagToChase = 1;
    private int m_GoalToChase = 1;
    private PhotonTrackedTarget m_lastTarget;

    private final ProfiledPIDController xController = 
        new ProfiledPIDController(
            VisionConstants.kXControllerP, 
            0, 
            VisionConstants.kXControllerD, XConstraints);

    private final ProfiledPIDController yController = 
        new ProfiledPIDController(
            VisionConstants.kYControllerP, 
            0, 
            VisionConstants.kYControllerD, YConstraints);

    private final ProfiledPIDController omegaController = 
        new ProfiledPIDController(
            VisionConstants.kOmegaControllerP, 
            0, 
            VisionConstants.kOmegaControllerD, OmegaConstraints);

    public TrackApriltag(){
        m_camera = RobotContainer.m_visionSubsystem.getCamera();
        m_driveSubsystem = RobotContainer.m_driveSubsystem;
        m_robotPoserProvider = RobotContainer.m_driveSubsystem::getPose;

        xController.setTolerance(0.04);
        yController.setTolerance(0.4);
        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(RobotContainer.m_driveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        /* 
        if (m_camera.getLatestResult().hasTargets()) {
            tagToChase =  m_camera.getLatestResult().getBestTarget().getFiducialId();
        } else {
            isFinished[0] = true;
            isFinished[1] = true;
            isFinished[2] = true;
        }
        var robotPose = m_robotPoserProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        xController.reset(robotPose.getY());
        var cameraRes = m_camera.getLatestResult();
        if (cameraRes.hasTargets()) {
            m_lastTarget = cameraRes.getBestTarget();
            tagToChase =  m_lastTarget.getFiducialId();
        } else {
            m_lastTarget = null;
        }
        
    }

    @Override
    public void execute() {
        super.execute();
        var robotPose2d = m_robotPoserProvider.get();
        var robotPose3d = new Pose3d(robotPose2d);

        var cameraRes = m_camera.getLatestResult();
        if (cameraRes.hasTargets()) {
            var targetOpt = cameraRes.getTargets().stream()
                .filter(t -> t.getFiducialId() == tagToChase) 
                .filter(t -> t.equals(m_lastTarget))
                .findFirst();
            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                m_lastTarget = target;
                var cameraPose = robotPose3d.transformBy(VisionConstants.kTransformRobotToCam);
                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);
                var goalPose = targetPose.transformBy(goalPositions[m_GoalToChase]).toPose2d();
                SmartDashboard.putNumber("goal X", goalPose.getX());
                SmartDashboard.putNumber("goal Y", goalPose.getY());
                SmartDashboard.putNumber("goal Omega", goalPose.getRotation().getRadians());
                

                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                omegaController.setGoal(goalPose.getRotation().getRadians());
            }
        }
        
        if (m_lastTarget == null) {
            m_driveSubsystem.drive(0, 0, 0, false, false);
        } else {
            var xSpeed = xController.calculate(robotPose2d.getX());
            if (xController.atGoal()){
                xSpeed = 0;
            }
            var ySpeed = yController.calculate(robotPose2d.getY());
            if (yController.atGoal()){
                ySpeed = 0;
            }
            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()){
                omegaSpeed = 0;
            }
            m_driveSubsystem.drive(xSpeed, ySpeed, omegaSpeed, true, false);
            SmartDashboard.putNumber("X Speed apriltag", xSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_driveSubsystem.drive(0, 0, 0, false, false);
    } 

    public void setGoalToChase(int goal){
        m_GoalToChase = goal;
    }
    */
}
