package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    public void execute() {
        super.execute();
        m_elevator.setLevel(m_elevatorLevel);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.m_elevatorSubsystem.getController().atGoal();
    }
}
