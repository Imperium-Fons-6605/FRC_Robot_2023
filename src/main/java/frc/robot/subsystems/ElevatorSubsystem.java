package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Util.ImperiumSparkMax;
import frc.robot.Util.Constants.ElevatorConstants;

public class ElevatorSubsystem extends PIDSubsystem{
    private final ImperiumSparkMax m_master;
    private final ImperiumSparkMax m_slave;
    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }
    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }

    public ElevatorSubsystem(PIDController controller) {
        super(controller);
        m_master = new ImperiumSparkMax(
            ElevatorConstants.kElevatorMasterCANId, 
            MotorType.kBrushless, 
            false, 
            ImperiumSparkMax.EncoderTypes.RELATIVE, 
            getSetpoint(), getSetpoint(), getSetpoint(), getSetpoint(), getMeasurement(), getSetpoint(), 0);
        m_slave = new ImperiumSparkMax(
            ElevatorConstants.kElevatorSlaveCANId, 
            MotorType.kBrushless, 
            false, 
            ImperiumSparkMax.EncoderTypes.RELATIVE, 
            getSetpoint(), getSetpoint(), getSetpoint(), getSetpoint(), getMeasurement(), getSetpoint(), 0);
    }

    
    
}
