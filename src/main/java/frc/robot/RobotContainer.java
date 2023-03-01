
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import org.opencv.features2d.BFMatcher;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.PS4Controller;

public class RobotContainer {
  private double swervePot = 1;

  private Command m_testPathPlannerAuto;
  List<PathPlannerTrajectory> m_testPathPlannerTrajectory;

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  //public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  //public static final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  public static final PS4Controller m_PS4CommandsController = new PS4Controller(OIConstants.kDriverControllerPort);
  public static final GenericHID m_GenericCommandsController = new GenericHID(OIConstants.kDriverControllerPort);
  public static final CSBalanceCommand m_balanceCommand = new CSBalanceCommand();
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private String driverJoystickname;
  private double currentVoltage;



  public RobotContainer() {
    //driverJoystickname = DriverStation.getJoystickName(0);
    //SmartDashboard.putData(m_elevatorSubsystem);

    configureButtonBindings();

    createTrayectories();
    m_autoChooser.addOption("Balance command", m_balanceCommand);
  }

 
  private void configureButtonBindings() {
    final GenericHID m_GenericDriverController = new GenericHID(OIConstants.kDriverControllerPort);
      m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.drive(
              -MathUtil.applyDeadband(m_GenericDriverController.getRawAxis(OIConstants.kLogitechLeftYAxis) * swervePot * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_GenericDriverController.getRawAxis(OIConstants.kLogitechLeftXAxis) * swervePot * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_GenericDriverController.getRawAxis(OIConstants.kLogitechRightXAxis) * swervePot * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                true, true), m_driveSubsystem));
    
    new JoystickButton(m_GenericDriverController, Button.kTriangle.value)
      .whileTrue(new RunCommand(
          () -> m_driveSubsystem.setX(),
          m_driveSubsystem));
      
      new JoystickButton(m_GenericDriverController, Button.kR1.value).onTrue(new InstantCommand(() -> setSwervePot(0.25)));
      new JoystickButton(m_GenericDriverController, Button.kL1.value).onTrue(new InstantCommand(() -> setSwervePot(1)));
    /* 
    if (driverJoystickname == "Logitech Dual Action") {
      

      new JoystickButton(m_GenericDriverController, Button.kTriangle.value)
      .whileTrue(new RunCommand(
          () -> m_driveSubsystem.setX(),
          m_driveSubsystem));
      
      new JoystickButton(m_GenericDriverController, Button.kR1.value).onTrue(new InstantCommand(() -> setSwervePot(0.25)));
      new JoystickButton(m_GenericDriverController, Button.kL1.value).onTrue(new InstantCommand(() -> setSwervePot(1)));

    } else {
      final PS4Controller m_PS4DriverController = new PS4Controller(OIConstants.kDriverControllerPort);
      m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_PS4DriverController.getLeftY() * swervePot * ((currentVoltage * 10)/(12.0-10)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_PS4DriverController.getLeftX() * swervePot * ((currentVoltage * 10)/(12.0-10)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_PS4DriverController.getRightX() * swervePot * ((currentVoltage * 10)/(12.0-10)), OIConstants.kDriveDeadband),
                true, true), m_driveSubsystem));

      new JoystickButton(m_PS4DriverController, Button.kTriangle.value)
      .whileTrue(new RunCommand(
          () -> m_driveSubsystem.setX(),
          m_driveSubsystem));
      
      new JoystickButton(m_PS4DriverController, Button.kR1.value).onTrue(new InstantCommand(() -> setSwervePot(0.25)));
      new JoystickButton(m_PS4DriverController, Button.kL1.value).onTrue(new InstantCommand(() -> setSwervePot(1)));
    }
    */
  }

  public Command getAutonomousCommand() {

    Command m_auto = m_autoChooser.getSelected();
    if (m_auto.getName() == "Balance command"){
      return m_auto;
    } else {
      return m_auto.andThen(() -> m_driveSubsystem.drive(0, 0, 0, false, false));
    }
  }

  private void setSwervePot(double pot){
    swervePot = pot;
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
        new PIDConstants(4.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(2, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        m_driveSubsystem::setModuleStates, 
        AutoConstants.kEventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_driveSubsystem 
    );
    m_testPathPlannerAuto = autoBuilder.fullAuto(m_testPathPlannerTrajectory);


    m_autoChooser.setDefaultOption("Test PathPlaner Auto", m_testPathPlannerAuto);
    SmartDashboard.putData(m_autoChooser);
    

  }
}
