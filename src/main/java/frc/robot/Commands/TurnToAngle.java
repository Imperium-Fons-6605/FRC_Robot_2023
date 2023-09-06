package frc.robot.Commands;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.subsystems.SwerveDrive;
/*
public class TurnToAngle extends CommandBase{
    private final  ProfiledPIDController m_controller = new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(180, -180));
    private final SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(0.1, 0.2);
    private final SwerveDrive driveSubsystem;

    public TurnToAngle(double pGoal){
        m_controller.enableContinuousInput(-180, 180);
        m_controller.setGoal(pGoal);
        driveSubsystem = RobotContainer.m_driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double pidOutput = m_controller.calculate(driveSubsystem.getHeadingDegrees());
        double setpoint = m_controller.getSetpoint().velocity;
        double feedforwardOutput =  m_Feedforward.calculate(setpoint);
        driveSubsystem.drive(
            -MathUtil.applyDeadband(RobotContainer.m_PS4DriverController.getLeftY() * RobotContainer.m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(RobotContainer.m_PS4DriverController.getLeftX() * RobotContainer.m_driveSubsystem.getVoltagePot(), OIConstants.kDriveDeadband),
             (feedforwardOutput + pidOutput),
              true,
               true);
    }
    @Override
    public boolean isFinished() {
        return m_controller.atGoal();
    }
}
*/
