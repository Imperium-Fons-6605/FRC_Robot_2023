package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Util.Constants.ClawConstants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ClawSubsystem extends PIDSubsystem{
    private final VictorSPX angleVictorSPX = new VictorSPX(0);
    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(0);

    public ClawSubsystem() {
        super(new PIDController(
            ClawConstants.kClawAngleP,
            ClawConstants.kClawAngleI,
            ClawConstants.kClawAngleD));
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
    }
    
}
