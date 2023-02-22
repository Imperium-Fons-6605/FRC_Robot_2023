
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.DriveConstants;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class RobotContainer {
    private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    private SwerveControllerCommand m_testScratchAuto;
    private Command m_testPathPlannerAuto;
    Trajectory m_testScratchTrajectory;
    List<PathPlannerTrajectory> m_testPathPlannerTrajectory;
    PathPlannerTrajectory example;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


  public RobotContainer() {

    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    createTrayectories();
  }

 
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kTriangle.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  public Command getAutonomousCommand() {

    Command m_auto = m_autoChooser.getSelected();
    if (m_auto.getName() == "Test Scratch Auto") {
        m_robotDrive.resetOdometry(m_testScratchTrajectory.getInitialPose());
    } else {
        System.out.println("No Valid Auto Selected");
    }

    return m_auto.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }



  public void createTrayectories(){

    //Creates an scratch trayectory via Wpilib official classes

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory m_testScratchTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_testScratchAuto = new SwerveControllerCommand(
        m_testScratchTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


    //Creates a Trayectory via PathPlanner API 

    m_testPathPlannerTrajectory = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Put Cargo", new PrintCommand("Putting cargo..."));
    eventMap.put("Grab Cargo", new PrintCommand("Grabbing Cargo..."));
    eventMap.put("Balance", new PrintCommand("Balancing..."));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_robotDrive::getPose, 
        m_robotDrive::resetOdometry, 
        DriveConstants.kDriveKinematics, 
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        m_robotDrive::setModuleStates, 
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_robotDrive 
    );

    m_testPathPlannerAuto = autoBuilder.fullAuto(m_testPathPlannerTrajectory);

    m_autoChooser.setDefaultOption("Test Scratch Auto", m_testScratchAuto);
    m_autoChooser.addOption("Test PathPlanner Auto", m_testPathPlannerAuto);
    SmartDashboard.putData(m_autoChooser);
    

  }
}
