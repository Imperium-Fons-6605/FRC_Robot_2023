package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Util.Constants.ElevatorConstants;

public class ElevatorSubsystem extends ProfiledPIDSubsystem{
    private final CANSparkMax m_masterSparkMax = new CANSparkMax(ElevatorConstants.kElevatorMasterCANId, MotorType.kBrushless);
    private final CANSparkMax m_slaveSparkMax = new CANSparkMax(ElevatorConstants.kElevatorSlaveCANId, MotorType.kBrushless);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        ElevatorConstants.kElevatorSVolts,
        ElevatorConstants.kElevatorGVolts,
        ElevatorConstants.kElevatorVVoltSecperMeter);
    private final RelativeEncoder m_encoder = m_masterSparkMax.getEncoder();

    double prevSetpoint = 0;
    double setpoint;

    public ElevatorSubsystem() {
        super(
            new ProfiledPIDController(
                ElevatorConstants.kElevatorP,
                ElevatorConstants.kElevatorI,
                ElevatorConstants.kElevatorD,
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.kMaxVelocityMeterperSec,
                    ElevatorConstants.kMaxAcceleratioMeterperSecSquared))
            );
        
        m_slaveSparkMax.follow(m_masterSparkMax, true);
        m_encoder.setPositionConversionFactor( ElevatorConstants.kEncoderPositionFactor);
        m_encoder.setPosition(0);


    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putNumber("Position", getMeasurement());
        SmartDashboard.putBoolean("At Setpoint", getController().atSetpoint());
        setpoint = SmartDashboard.getNumber("Setpoint", 0);
        if (setpoint != prevSetpoint){
            prevSetpoint = setpoint;
            setGoal(setpoint);
        }

    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_masterSparkMax.setVoltage(output + feedforward);
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return m_encoder.getPosition();
    }   
}
