
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.CSBalanceCommand;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.DriveConstants;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.PS4Controller;

public class RobotContainer {

  private Command m_testPathPlannerAuto;
  List<PathPlannerTrajectory> m_testPathPlannerTrajectory;

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  //public static final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  public static final PS4Controller m_PS4CommandsController = new PS4Controller(OIConstants.kDriverControllerPort);
  public static final GenericHID m_GenericCommandsController = new GenericHID(OIConstants.kDriverControllerPort);
  public static final CSBalanceCommand m_balanceCommand = new CSBalanceCommand();
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();




  public RobotContainer() {
    //SmartDashboard.putData(m_elevatorSubsystem);

    configureButtonBindings();

    createTrayectories();
  }

 
  private void configureButtonBindings() {
    if (OIConstants.kDriverControllerType == "PS4"){
      final PS4Controller m_PS4DriverController = new PS4Controller(OIConstants.kDriverControllerPort);
      m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_PS4DriverController.getLeftY() * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_PS4DriverController.getLeftX() * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_PS4DriverController.getRightX() * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                true, true), m_driveSubsystem));

      new JoystickButton(m_PS4DriverController, Button.kTriangle.value)
      .whileTrue(new RunCommand(
          () -> m_driveSubsystem.setX(),
          m_driveSubsystem));

      new JoystickButton(m_PS4DriverController, Button.kCircle.value).onTrue(Commands.runOnce(
        () -> {
          m_elevatorSubsystem.setGoal(30);
          m_elevatorSubsystem.enable();
        },
        m_elevatorSubsystem));
      new JoystickButton(m_PS4DriverController, Button.kCross.value).onTrue(Commands.runOnce(
        () -> {
          m_elevatorSubsystem.setGoal(10);
          m_elevatorSubsystem.enable();
        },
        m_elevatorSubsystem));
      
      new JoystickButton(m_PS4DriverController, Button.kR1.value).onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(true)));
      new JoystickButton(m_PS4DriverController, Button.kL1.value).onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(false)));

    } else {

      final GenericHID m_GenericDriverController = new GenericHID(OIConstants.kDriverControllerPort);
      m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.drive(
              -MathUtil.applyDeadband(m_GenericDriverController.getRawAxis(OIConstants.kLogitechLeftYAxis)  * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_GenericDriverController.getRawAxis(OIConstants.kLogitechLeftXAxis)  * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_GenericDriverController.getRawAxis(OIConstants.kLogitechRightXAxis)  * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                true, true), m_driveSubsystem));
    
    new JoystickButton(m_GenericDriverController, Button.kTriangle.value)
      .whileTrue(new RunCommand(
          () -> m_driveSubsystem.setX(),
          m_driveSubsystem));
      
      new JoystickButton(m_GenericDriverController, Button.kR1.value).onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(true)));
      new JoystickButton(m_GenericDriverController, Button.kL1.value).onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(false)));
    }
  }

  public Command getAutonomousCommand() {

    Command m_auto = m_autoChooser.getSelected();
    if (m_auto.getName() == "Balance command"){
      return m_auto;
    } else {
      return m_auto.andThen(() -> m_driveSubsystem.setX());
    }
  }




  public void createTrayectories(){

    //Creates a Trayectory via PathPlanner API 

    m_testPathPlannerTrajectory = PathPlanner.loadPathGroup("TestPath", AutoConstants.kAutoConstraints);

    AutoConstants.kEventMap.put("Put Cargo", new PrintCommand("Putting cargo..."));
    AutoConstants.kEventMap.put("Grab Cargo", new PrintCommand("Grabbing Cargo..."));
    AutoConstants.kEventMap.put("Balance", new PrintCommand("Balancing..."));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_driveSubsystem::getPose, 
        m_driveSubsystem::resetOdometry, 
        DriveConstants.kDriveKinematics, 
        new PIDConstants(3, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        m_driveSubsystem::setModuleStates, 
        AutoConstants.kEventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_driveSubsystem 
    );
    m_testPathPlannerAuto = autoBuilder.fullAuto(m_testPathPlannerTrajectory);


    m_autoChooser.setDefaultOption("Test PathPlaner Auto", m_testPathPlannerAuto);
    m_autoChooser.addOption("Balance command", m_balanceCommand);
    SmartDashboard.putData(m_autoChooser);

    SmartDashboard.putData("Test Path Planner Auto", new InstantCommand(() -> m_testPathPlannerAuto.schedule()));
    SmartDashboard.putData("CSBalanceCommand", new InstantCommand(() -> m_balanceCommand.schedule()));

    PathPlannerServer.startServer(6605);
    

  }
}
