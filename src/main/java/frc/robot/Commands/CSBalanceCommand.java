package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class CSBalanceCommand extends CommandBase{
    private DriveSubsystem m_DriveSubsystem;

  private double X_error;
  private double X_currentAngle;
  private double X_drivePower;

  private boolean isDocked = false;

  public CSBalanceCommand() {
    this.m_DriveSubsystem = RobotContainer.m_driveSubsystem;
    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() { 
    isDocked = false;
  }

  @Override
  public void execute() {
    X_currentAngle = -m_DriveSubsystem.getPigeonGyro().getPitch();

    X_error = (AutoConstants.kBalanceGoalDegrees - X_currentAngle);
    X_drivePower = (-Math.min(AutoConstants.kBalancedDrivekP * X_error, (AutoConstants.kBalancedDrivekP * 45))/9);

    /*if (drivePower < 0) {
      drivePower *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }
    */

    if (Math.abs(X_drivePower) > 0.2) {
      X_drivePower = Math.copySign(0.2, X_drivePower);
    }

    m_DriveSubsystem.drive(X_drivePower,0, 0, true, true);;
    /* 
    SmartDashboard.putNumber("X_Current Angle", X_currentAngle);
    SmartDashboard.putNumber("X_Error", X_error);
    SmartDashboard.putNumber("X_Drive Power", X_drivePower);
    */
    if (Math.abs(X_error) < AutoConstants.kBalanceAngleTresholdDegrees) {
      m_DriveSubsystem.setX();
      isDocked = true;
      LEDSubsystem.getInstance().setRainbow();
    } else{
      LEDSubsystem.getInstance().setCustomColor(0,255,255);

    }
   

  }

  @Override
  public boolean isFinished() {
    return isDocked; 
  }
}
