
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Autos;
import frc.robot.Commands.CSBalanceCommand;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;


public class RobotContainer {
  //Subsystems
  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  //Comands
  public static final Autos m_autosFactory = new Autos();
  private SendableChooser<Integer> m_positionChooser = new SendableChooser<>();
  private SendableChooser<Boolean> m_onBalanceChooser = new SendableChooser<>();
  //Joysticks
  public static final CommandGenericHID m_GenericCommandsController = new CommandGenericHID(OIConstants.kCommandsControllerPort);
  public static final CommandPS4Controller m_PS4DriverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);

  



  public RobotContainer() {
    AutoConstants.kEventMap.put("GoDown", new InstantCommand(() -> m_elevatorSubsystem.setLevel(0)));
    AutoConstants.kEventMap.put("GoUp", new InstantCommand(() -> m_elevatorSubsystem.setLevel(2)));
    AutoConstants.kEventMap.put("PutUpCargo", new PrintCommand("Putting Cargo up..."));
    AutoConstants.kEventMap.put("GrabCargo", new PrintCommand("Grabbing Cargo..."));
    AutoConstants.kEventMap.put("Balance", new CSBalanceCommand());
    m_autosFactory.buildAutos();
    configureButtonBindings();
  }

 
  private void configureButtonBindings() {
    //Driver bindings
    m_driveSubsystem.setDefaultCommand(
      new RunCommand(
          () -> m_driveSubsystem.drive(
              -MathUtil.applyDeadband(m_PS4DriverController.getLeftY() * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_PS4DriverController.getLeftX() * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_PS4DriverController.getRightX() * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
              true, true), m_driveSubsystem));
    m_PS4DriverController.triangle()
    .whileTrue(new RunCommand(
        () -> m_driveSubsystem.setX(),
        m_driveSubsystem));
    m_PS4DriverController.R1().onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(true)));
    m_PS4DriverController.L1().onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(false)));

    //Commands bindings
    m_GenericCommandsController.povUp().onTrue(new InstantCommand(() -> m_elevatorSubsystem.goUp()));
    m_GenericCommandsController.povDown().onTrue(new InstantCommand(() -> m_elevatorSubsystem.goDown()));
    m_GenericCommandsController.button(4).onTrue(
      new InstantCommand(() -> m_elevatorSubsystem.toggleManual()));
  }

  public Command getAutonomousCommand() {
    return m_autosFactory.getAuto(m_positionChooser.getSelected(), m_onBalanceChooser.getSelected());
  }
}
