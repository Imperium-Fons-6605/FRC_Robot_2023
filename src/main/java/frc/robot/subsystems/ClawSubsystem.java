package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Util.Constants.ClawConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ClawSubsystem extends ProfiledPIDSubsystem{
    private final VictorSPX m_angleVictor = new VictorSPX(ClawConstants.kClawAngleCANId);
    private final VictorSPX m_masterExpulsorVictor = new VictorSPX(ClawConstants.kClawMasterExpulsorCANId);
    private final VictorSPX m_slaveExpulsorVictor = new VictorSPX(ClawConstants.kClawSlaveExpulsorCANId);
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(ClawConstants.kClawAngleEncoderDIOPort);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
        ClawConstants.kClawSVolts, 
        ClawConstants.kClawGVolts,
        ClawConstants.kClawVVoltSecPerRad);

    public ClawSubsystem() {
        super(new ProfiledPIDController(
            ClawConstants.kClawAngleP,
            ClawConstants.kClawAngleI,
            ClawConstants.kClawAngleD,
            new TrapezoidProfile.Constraints(
                ClawConstants.kClawMaxVelocityRadPerSec, 
                ClawConstants.kClawMaxAccelerationRadPerSecSquared)
            )
        );
        m_angleEncoder.setDistancePerRotation(ClawConstants.kWristEncoderPositionFactor);
        m_masterExpulsorVictor.setInverted(true);
        m_slaveExpulsorVictor.setInverted(false);
        m_slaveExpulsorVictor.follow(m_masterExpulsorVictor,FollowerType.PercentOutput);
        getController().setTolerance(0.08);
    }

    @Override
    protected double getMeasurement() {
        return m_angleEncoder.getAbsolutePosition();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = m_feedforward.calculate( getMeasurement(),setpoint.velocity);
        double totalOutput = (feedforward + output);
        /* 
        SmartDashboard.putBoolean("At Setpoint", getController().atSetpoint());
        SmartDashboard.putNumber("goal", getController().getGoal().position);
        SmartDashboard.putNumber("feedforward output", feedforward);
        SmartDashboard.putNumber("setpoint Position", setpoint.position);
        SmartDashboard.putNumber("setpoint Velocity", setpoint.velocity);
        SmartDashboard.putNumber("measurment", getMeasurement());
        SmartDashboard.putNumber("PID Output", output);
        SmartDashboard.putNumber("Applied output", totalOutput);
        */
        m_angleVictor.set(ControlMode.PercentOutput, totalOutput);
    }

    public void setClawPercentOutput(double percentOutput){
        m_masterExpulsorVictor.set(ControlMode.PercentOutput, percentOutput);
    }    
}
