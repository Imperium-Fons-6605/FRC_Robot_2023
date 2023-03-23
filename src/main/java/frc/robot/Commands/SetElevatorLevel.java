package frc.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorLevel extends CommandBase{
    private int m_elevatorLevel;
    private ElevatorSubsystem m_elevator = RobotContainer.m_elevatorSubsystem;

    public SetElevatorLevel (int level){
        m_elevatorLevel = level;
        addRequirements(RobotContainer.m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_elevator.setLevel(m_elevatorLevel);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.m_elevatorSubsystem.getController().atGoal();
    }
}
