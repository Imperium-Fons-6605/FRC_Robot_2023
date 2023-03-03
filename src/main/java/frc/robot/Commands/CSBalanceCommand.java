package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CSBalanceCommand extends CommandBase{
    private DriveSubsystem m_DriveSubsystem;

  private double X_error;
  private double X_currentAngle;
  private double X_drivePower;

  private int executions = 0;

  private boolean isDocked = false;


  // Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace 
  public CSBalanceCommand() {
    this.m_DriveSubsystem = RobotContainer.m_driveSubsystem;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    isDocked = false;
    SmartDashboard.putNumber("Balance Executions", executions);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    executions++;
    SmartDashboard.putNumber("Balance Executions", executions);
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    //X_currentAngle = -RobotContainer.m_GenericCommandsController.getRawAxis(3) * 45;
    X_currentAngle = -m_DriveSubsystem.getPigeonGyro().getPitch();

    X_error = (AutoConstants.kBalanceGoalDegrees - X_currentAngle);
    X_drivePower = (-Math.min(AutoConstants.kBalancedDrivekP * X_error, (AutoConstants.kBalancedDrivekP * 45))/9);

    // Our robot needed an extra pus+h to drive up in reverse, probably due to weight imbalances
    /*if (drivePower < 0) {
      drivePower *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }
    */

    // Limit the max power
    if (Math.abs(X_drivePower) > 0.4) {
      X_drivePower = Math.copySign(0.4, X_drivePower);
    }

    m_DriveSubsystem.drive(X_drivePower,0, 0, true, true);;
    
    // Debugging Print Statments
    SmartDashboard.putNumber("X_Current Angle", X_currentAngle);
    SmartDashboard.putNumber("X_Error", X_error);
    SmartDashboard.putNumber("X_Drive Power", X_drivePower);
    if (Math.abs(X_error) < AutoConstants.kBalanceAngleTresholdDegrees) {
      m_DriveSubsystem.setX();
      isDocked = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDocked; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}
