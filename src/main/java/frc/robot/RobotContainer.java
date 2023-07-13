
package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Autos;
import frc.robot.Commands.CSBalanceCommand;
import frc.robot.Commands.SetElevatorLevel;
import frc.robot.Commands.SetExtensorLevel;
import frc.robot.Commands.TrackApriltag;
import frc.robot.Commands.TurnToAngle;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.MechanismSim;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  //Subsystems
  public static final SwerveDrive m_driveSubsystem = new SwerveDrive();
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public static final ExtensorSubsystem m_extensorSubsystem = new ExtensorSubsystem();
  public static final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  public static final FieldSim m_field = new FieldSim(m_driveSubsystem);
  public static final MechanismSim m_mechSim = new MechanismSim(m_elevatorSubsystem, m_extensorSubsystem, m_clawSubsystem);

  //Comands
  public static final Autos m_autosFactory = new Autos();
  private SendableChooser<Integer> m_positionChooser = new SendableChooser<>();
  private SendableChooser<Boolean> m_onBalanceChooser = new SendableChooser<>();
  private SendableChooser<Boolean> m_onTrayectoryChooser = new SendableChooser<>();
  private TrackApriltag m_trackApriltagCommand = new TrackApriltag();
  //Joysticks
  public static final CommandXboxController m_XboxCommandsController = new CommandXboxController(OIConstants.kCommandsControllerPort);
  public static final CommandXboxController m_SimDriveController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static final CommandGenericHID m_DriverController = new CommandGenericHID(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    
    AutoConstants.kEventMap.put("GoDown", new InstantCommand(() -> m_elevatorSubsystem.setLevel(0)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(0))).andThen(new InstantCommand(() -> m_clawSubsystem.setLevel(0))));
    AutoConstants.kEventMap.put("GoUp", new InstantCommand(() -> m_elevatorSubsystem.setLevel(2)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(2))).andThen(new InstantCommand(() -> m_clawSubsystem.setLevel(2))));
    AutoConstants.kEventMap.put("PutCargo", new InstantCommand(() -> m_clawSubsystem.putCargo()));
    AutoConstants.kEventMap.put("GrabCargo", new InstantCommand(() -> m_clawSubsystem.grabCargo()));
    AutoConstants.kEventMap.put("StopIntake", new InstantCommand(() ->m_clawSubsystem.stopClaw()));
    AutoConstants.kEventMap.put("ExtendClaw", new InstantCommand(() -> m_clawSubsystem.setLevel(4)));
    AutoConstants.kEventMap.put("WaitForElevator", new WaitCommand(1.7));
    AutoConstants.kEventMap.put("WaitForIntake", new WaitCommand(1));
    AutoConstants.kEventMap.put("WaitForClaw", new WaitCommand(1));
    AutoConstants.kEventMap.put("setCube", new InstantCommand(() -> setCargoIsCube(true)));
    AutoConstants.kEventMap.put("setCone", new InstantCommand(() -> setCargoIsCube(false)));
    AutoConstants.kEventMap.put("Balance", new CSBalanceCommand());
    m_autosFactory.buildAutos();
    m_positionChooser.setDefaultOption("1", 1);
    m_positionChooser.addOption("2", 2);
    m_positionChooser.addOption("3", 3);
    m_onBalanceChooser.addOption("Yes", true);
    m_onBalanceChooser.addOption("No", false);
    m_onTrayectoryChooser.addOption("Yes", true);
    m_onTrayectoryChooser.addOption("No", false);
    SmartDashboard.putData("Position Chooser", m_positionChooser);
    SmartDashboard.putData("Balance?", m_onBalanceChooser);
    SmartDashboard.putData("Trayectory?", m_onTrayectoryChooser);
    m_field.initSim();
    configureButtonBindings();
  }

 
  private void configureButtonBindings() {
    if (Robot.isSimulation()){
      //Driver bindings
      m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.drive(
                MathUtil.applyDeadband(squareJoystickInput(m_SimDriveController.getLeftX()) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(squareJoystickInput(m_SimDriveController.getLeftY()) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(squareJoystickInput(m_SimDriveController.getRightX()) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                true,true), m_driveSubsystem));
      
      m_SimDriveController.x().onTrue(new InstantCommand(() -> m_driveSubsystem.zeroHeading()));
      m_SimDriveController.rightBumper().onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(true)));
      m_SimDriveController.leftBumper().onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(false)));
      //m_SimDriveController.povUp().onTrue(new TurnToAngle(0));
      //m_SimDriveController.povLeft().onTrue(new TurnToAngle(90));
      //m_SimDriveController.povDown().onTrue(new TurnToAngle(180));
      //m_SimDriveController.povRight().onTrue(new TurnToAngle(-90));
    } else {
      m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(squareJoystickInput(m_DriverController.getRawAxis(1)) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(squareJoystickInput(m_DriverController.getRawAxis(0)) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(squareJoystickInput(m_DriverController.getRawAxis(4)) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
                true, true), m_driveSubsystem));

      m_DriverController.button(Constants.OIConstants.kLogitechUpButton)
      .whileTrue(new RunCommand(
          () -> m_driveSubsystem.setX(),
          m_driveSubsystem));
      m_DriverController.button(3).onTrue(new InstantCommand(() -> m_driveSubsystem.zeroHeading()));
      m_DriverController.button(Constants.OIConstants.kLogitechR1).onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(true)));
      m_DriverController.button(Constants.OIConstants.kLogitechL1).onTrue(new InstantCommand(() -> m_driveSubsystem.reductVelocity(false)));
      //m_PS4DriverController.povLeft().onTrue(new InstantCommand(()-> m_trackApriltagCommand.setGoalToChase(2)));
      //m_PS4DriverController.povDown().onTrue(new InstantCommand(()-> m_trackApriltagCommand.setGoalToChase(1)));
      //m_PS4DriverController.povRight().onTrue(new InstantCommand(()-> m_trackApriltagCommand.setGoalToChase(0)));
      //m_PS4DriverController.R2().whileTrue(m_trackApriltagCommand);
      //m_PS4DriverController.povUp().onTrue(new TurnToAngle(0));
      //m_PS4DriverController.povLeft().onTrue(new TurnToAngle(90));
      //m_PS4DriverController.povDown().onTrue(new TurnToAngle(180));
      //m_PS4DriverController.povRight().onTrue(new TurnToAngle(-90));
    }

      //Commands bindings
      m_XboxCommandsController.a().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevel(0)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(0))).andThen(new InstantCommand(() -> m_clawSubsystem.setLevel(0))));
      m_XboxCommandsController.b().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevel(1)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(1))).andThen(new InstantCommand(() -> m_clawSubsystem.setLevel(1))));
      m_XboxCommandsController.y().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevel(2)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(2))).andThen(new InstantCommand(() -> m_clawSubsystem.setLevel(2))));
      m_XboxCommandsController.x().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevel(3)).andThen(new InstantCommand(() -> m_extensorSubsystem.setExtension(3))).andThen(new InstantCommand(() -> m_clawSubsystem.setLevel(3))));
      m_XboxCommandsController.rightStick().onTrue(new InstantCommand(() -> m_clawSubsystem.setLevel(4))); 
      m_XboxCommandsController.rightBumper().onTrue(new InstantCommand(() -> setCargoIsCube(false)));
      m_XboxCommandsController.leftBumper().onTrue(new InstantCommand(() -> setCargoIsCube(true)));
      m_XboxCommandsController.leftTrigger().whileTrue(new InstantCommand(() -> m_clawSubsystem.grabCargo()));
      m_XboxCommandsController.rightTrigger().whileTrue(new InstantCommand(() -> m_clawSubsystem.putCargo()));
      m_XboxCommandsController.leftTrigger().or(m_XboxCommandsController.rightTrigger()).onFalse(new InstantCommand(() -> m_clawSubsystem.stopClaw()));

    //} else{
      /*
      GenericHID m_GenericHID = new GenericHID(0);
      m_driveSubsystem.setDefaultCommand(
      new RunCommand(
          () -> m_driveSubsystem.drive(
              -MathUtil.applyDeadband(m_GenericHID.getRawAxis(1) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_GenericHID.getRawAxis(0) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_GenericHID.getRawAxis(2) * m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
              true, true), m_driveSubsystem));
      */
    //}
  }


  public Command getAutonomousCommand() {
    return m_autosFactory.getAuto(m_positionChooser.getSelected(), m_onBalanceChooser.getSelected(), m_onTrayectoryChooser.getSelected());
  }
  
  public void setCargoIsCube(boolean pIsCube){
    m_elevatorSubsystem.setIsCargoCube(pIsCube);
    m_clawSubsystem.setIsCargoCube(pIsCube);
    m_extensorSubsystem.setIsCargoCube(pIsCube);
  }

  public double squareJoystickInput(double input){
    return Math.copySign((input*input), input);
  }

  public void periodic(){
    m_field.periodic();
    //m_mechSim.periodic();
  }

  public void simulationPeriodic(){
    //m_mechSim.simulationPeriodic();
  }
}
