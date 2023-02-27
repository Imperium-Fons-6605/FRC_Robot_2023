
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.DriveConstants;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.PS4Controller;

public class RobotContainer {
    private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    private Command m_testPathPlannerAuto;
    List<PathPlannerTrajectory> m_testPathPlannerTrajectory;

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  //public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  //public static final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  public static final PS4Controller m_driverController = new PS4Controller(0);
  //public static final GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);


  public RobotContainer() {
    SmartDashboard.putBoolean("Rate limiter", true);
    //SmartDashboard.putData(m_elevatorSubsystem);

    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.drive(
              /* 
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              true, true), m_driveSubsystem));
              */
            
                -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kLogitechLeftYAxis), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kLogitechLeftXAxis), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kLogitechRightXAxis), OIConstants.kDriveDeadband),
                true, SmartDashboard.getBoolean("Rate limiter", true)), m_driveSubsystem));
              

    createTrayectories();
  }

 
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kTriangle.value)
        .whileTrue(new RunCommand(
            () -> m_driveSubsystem.setX(),
            m_driveSubsystem));
    
    /*new JoystickButton(m_driverController, Button.kCircle.value)
    .onTrue(
        Commands.runOnce(
            () -> {
              m_elevatorSubsystem.setGoal(30); //goal in cm)
              m_elevatorSubsystem.enable();;
            },
            m_elevatorSubsystem)));
    */
  }

  public Command getAutonomousCommand() {

    Command m_auto = m_autoChooser.getSelected();
    return m_auto.andThen(() -> m_driveSubsystem.drive(0, 0, 0, false, false));
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
