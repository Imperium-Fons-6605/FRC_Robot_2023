package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
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
        SmartDashboard.putNumber("Setpoint", 0);
        m_masterSparkMax.restoreFactoryDefaults();
        m_slaveSparkMax.restoreFactoryDefaults();
        m_masterSparkMax.setSmartCurrentLimit(50);
        m_slaveSparkMax.setSmartCurrentLimit(50);
        m_masterSparkMax.setIdleMode(IdleMode.kBrake);
        m_slaveSparkMax.setIdleMode(IdleMode.kBrake);

        m_masterSparkMax.setInverted(true);

        m_slaveSparkMax.follow(m_masterSparkMax, false);

        m_encoder.setPositionConversionFactor( ElevatorConstants.kEncoderPositionFactor);
        m_encoder.setVelocityConversionFactor( ElevatorConstants.kEncoderVelocityFactor);
        m_encoder.setPosition(0);
        m_slaveSparkMax.burnFlash();
        m_masterSparkMax.burnFlash();

    }

    @Override
    public void periodic() {
        super.periodic();
        enable();

        setGoal(5);
        SmartDashboard.putBoolean("At Setpoint", getController().atSetpoint());
        SmartDashboard.putNumber("goal", getController().getGoal().position);

    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        double totalOutput = (feedforward + output);
        SmartDashboard.putNumber("feedforward output", feedforward);
        SmartDashboard.putNumber("setpoint position", setpoint.position);
        SmartDashboard.putNumber("setpoint velocity", setpoint.velocity);
        SmartDashboard.putNumber("PID Output", output);
        SmartDashboard.putNumber("measurment", getMeasurement());
        SmartDashboard.putNumber("Applied output", totalOutput);
        m_masterSparkMax.setVoltage(totalOutput);
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return m_encoder.getPosition();
    }   
}
