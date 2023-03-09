package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExtensorSubsystem;

public class SetExtensorLevel extends CommandBase{
    private int m_extensorLevel;
    private ExtensorSubsystem m_extensor = RobotContainer.m_extensorSubsystem;

    public SetExtensorLevel (int level){
        m_extensorLevel = level;
        addRequirements(RobotContainer.m_extensorSubsystem);
    }
    @Override
    public void execute() {
        super.execute();
        m_extensor.setExtension(m_extensorLevel);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.m_extensorSubsystem.getController().atGoal();
    }
}
