
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Autos;
import frc.robot.Commands.CSBalanceCommand;
import frc.robot.Commands.SetElevatorLevel;
import frc.robot.Commands.SetExtensorLevel;
import frc.robot.Commands.TrackApriltag;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  //Subsystems
  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public static final ExtensorSubsystem m_extensorSubsystem = new ExtensorSubsystem();
  //public static final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  //Comands
  public static final Autos m_autosFactory = new Autos();
  private SendableChooser<Integer> m_positionChooser = new SendableChooser<>();
  private SendableChooser<Boolean> m_onBalanceChooser = new SendableChooser<>();
  private TrackApriltag m_trackApriltagCommand = new TrackApriltag();
  //Joysticks
  public static final CommandXboxController m_XboxCommandsController = new CommandXboxController(OIConstants.kCommandsControllerPort);
  public static final CommandPS4Controller m_PS4DriverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);

  private int m_prevLevel = 0;

  



  public RobotContainer() {
    AutoConstants.kEventMap.put("GoDown", new SetElevatorLevel(0).alongWith(new SetExtensorLevel(0)));
    AutoConstants.kEventMap.put("GoUp", new SetElevatorLevel(2).alongWith(new SetExtensorLevel(2)));
    //AutoConstants.kEventMap.put("PutUpCargo", new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(-0.5)).withTimeout(1).andThen(new InstantCommand(() ->m_clawSubsystem.setClawPercentOutput(0))));
    //AutoConstants.kEventMap.put("GrabCargo", new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(0.5)).withTimeout(1).andThen(new InstantCommand(() ->m_clawSubsystem.setClawPercentOutput(0))));
    AutoConstants.kEventMap.put("Balance", new CSBalanceCommand());
    m_autosFactory.buildAutos();
    m_positionChooser.setDefaultOption("1", 1);
    m_positionChooser.addOption("2", 2);
    m_positionChooser.addOption("3", 3);
    m_onBalanceChooser.addOption("Yes", true);
    m_onBalanceChooser.addOption("No", false);
    SmartDashboard.putData("Position Chooser", m_positionChooser);
    SmartDashboard.putData("Balance?", m_onBalanceChooser);
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
    m_PS4DriverController.povLeft().onTrue(new InstantCommand(()-> m_trackApriltagCommand.setGoalToChase(2)));
    m_PS4DriverController.povDown().onTrue(new InstantCommand(()-> m_trackApriltagCommand.setGoalToChase(1)));
    m_PS4DriverController.povRight().onTrue(new InstantCommand(()-> m_trackApriltagCommand.setGoalToChase(0)));
    m_PS4DriverController.R2().whileTrue(m_trackApriltagCommand);

    //Commands bindings
    //m_XboxCommandsController.a().onTrue(new SetElevatorLevel(0).alongWith(new SetExtensorLevel(0)));
    //m_XboxCommandsController.b().onTrue(new SetElevatorLevel(1).alongWith(new SetExtensorLevel(1)));
    //m_XboxCommandsController.y().onTrue(new SetElevatorLevel(2).alongWith(new SetExtensorLevel(2)));
    /*
    m_XboxCommandsController.a().onTrue(new InstantCommand(() -> m_extensorSubsystem.setExtension(0)));
    m_XboxCommandsController.b().onTrue(new InstantCommand(() -> m_extensorSubsystem.setExtension(1)));
    m_XboxCommandsController.y().onTrue(new InstantCommand(() -> m_extensorSubsystem.setExtension(2)));
    */
    
    m_XboxCommandsController.a().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevel(0)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(0))));
    m_XboxCommandsController.b().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevel(1)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(1))));
    m_XboxCommandsController.y().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevel(2)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(2))));
    
    m_XboxCommandsController.rightBumper().onTrue(
      new InstantCommand(() -> setManual(true)).andThen(new PrintCommand("manual disabled")));
    m_XboxCommandsController.leftBumper().onTrue(
      new InstantCommand(() -> setManual(false)).andThen(new PrintCommand("manual enabled")));
    //m_XboxCommandsController.leftTrigger().whileTrue(new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(-0.5)));
    //m_XboxCommandsController.rightTrigger().whileTrue(new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(0.5)));
    //m_XboxCommandsController.leftTrigger().or(m_XboxCommandsController.rightTrigger()).onFalse(new InstantCommand(() -> m_clawSubsystem.setClawPercentOutput(0)));
  }


  public Command getAutonomousCommand() {
    return m_autosFactory.getAuto(m_positionChooser.getSelected(), m_onBalanceChooser.getSelected());
  }
  

  public void setManual(boolean pisManual){
      OIConstants.isManual = pisManual;
      if(pisManual = true){
        m_elevatorSubsystem.disable();
        m_extensorSubsystem.disable();
        //m_clawSubsystem.disable();
      } else {
        m_elevatorSubsystem.enable();
        m_extensorSubsystem.enable();
        //m_clawSubsystem.enable();
      }
  }
}
