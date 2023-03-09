package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawSubsystem;

public class SetClawAngle extends CommandBase{
    private int m_clawLevel;
    private ClawSubsystem m_claw = RobotContainer.m_clawSubsystem;

    public SetClawAngle(int level){
        m_clawLevel = level;
        addRequirements(RobotContainer.m_elevatorSubsystem);
    }
    @Override
    public void execute() {
        super.execute();
        m_claw.setGoal(m_clawLevel);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.m_clawSubsystem.getController().atGoal();
    }
}
