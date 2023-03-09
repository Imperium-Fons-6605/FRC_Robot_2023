
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Autos;
import frc.robot.Commands.CSBalanceCommand;
import frc.robot.Commands.SetElevatorLevel;
import frc.robot.Commands.SetExtensorLevel;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensorSubsystem;
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
  public static final ExtensorSubsystem m_extensorSubsystem = new ExtensorSubsystem();
  public static final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

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
    m_positionChooser.setDefaultOption("1", 1);
    m_positionChooser.addOption("2", 2);
    m_positionChooser.addOption("3", 3);
    m_onBalanceChooser.addOption("Yes", true);
    m_onBalanceChooser.addOption("No", false);
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
    m_GenericCommandsController.button(OIConstants.kLogitechDownButton).onTrue(new SetElevatorLevel(0).alongWith(new SetExtensorLevel(0)));
    m_GenericCommandsController.button(OIConstants.kLogitechRightButton).onTrue(new SetElevatorLevel(1).alongWith(new SetExtensorLevel(1)));
    m_GenericCommandsController.button(OIConstants.kLogitechUpButton).onTrue(new SetElevatorLevel(2).alongWith(new SetExtensorLevel(2)));
    m_GenericCommandsController.button(OIConstants.kLogitechR1).onTrue(
      new InstantCommand(() -> m_elevatorSubsystem.setManual(false)).alongWith(new InstantCommand(() -> m_extensorSubsystem.setManual(false))));
    m_GenericCommandsController.button(OIConstants.kLogitechL1).onTrue(
      new InstantCommand(() -> m_elevatorSubsystem.setManual(true)).alongWith(new InstantCommand(() -> m_extensorSubsystem.setManual(true))));
    m_GenericCommandsController.button(OIConstants.kLogitechL2).whileTrue(new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(-0.2)));
    m_GenericCommandsController.button(OIConstants.kLogitechR2).whileTrue(new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(0.2)));
    m_GenericCommandsController.button(OIConstants.kLogitechL2).or(m_GenericCommandsController.button(OIConstants.kLogitechR2)).onFalse(new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(0)));
  }

  public Command getAutonomousCommand() {
    return m_autosFactory.getAuto(m_positionChooser.getSelected(), m_onBalanceChooser.getSelected());
  }
}
