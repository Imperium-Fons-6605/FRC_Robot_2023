package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.OIConstants;

public class ElevatorSubsystem extends ProfiledPIDSubsystem{
    private final CANSparkMax m_masterSparkMax = new CANSparkMax(ElevatorConstants.kElevatorMasterCANId, MotorType.kBrushless);
    private final CANSparkMax m_slaveSparkMax = new CANSparkMax(ElevatorConstants.kElevatorSlaveCANId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_masterSparkMax.getEncoder();
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        ElevatorConstants.kElevatorSVolts,
        ElevatorConstants.kElevatorGVolts,
        ElevatorConstants.kElevatorVVoltSecperCm);
    private final SlewRateLimiter m_dirLimiter = new SlewRateLimiter(ElevatorConstants.kDirectionSlewRate);

    private boolean isManual = false;
    private int m_elevatorLevel = 0;

    public ElevatorSubsystem() {
        super(new ProfiledPIDController(
            ElevatorConstants.kElevatorP,
            ElevatorConstants.kElevatorI,
            ElevatorConstants.kElevatorD, 
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxVelocityCmperSec, 
                ElevatorConstants.kMaxAcceleratioCmperSecSquared)
        )
        );
        
        m_masterSparkMax.restoreFactoryDefaults();
        m_slaveSparkMax.restoreFactoryDefaults();

        m_masterSparkMax.setSmartCurrentLimit(50);
        m_slaveSparkMax.setSmartCurrentLimit(50);
        m_masterSparkMax.setIdleMode(IdleMode.kBrake);
        m_slaveSparkMax.setIdleMode(IdleMode.kBrake);

        m_masterSparkMax.setInverted(false);

        m_slaveSparkMax.follow(m_masterSparkMax, false);

        m_encoder.setPositionConversionFactor( ElevatorConstants.kEncoderPositionFactor);
        m_encoder.setVelocityConversionFactor( ElevatorConstants.kEncoderVelocityFactor);
        m_encoder.setPosition(0);
        
        m_slaveSparkMax.burnFlash();
        m_masterSparkMax.burnFlash();

        getController().setTolerance(2);

        SmartDashboard.putData(new InstantCommand(() -> m_encoder.setPosition(0)));
        SmartDashboard.putNumber("Elevator P", ElevatorConstants.kElevatorP);
        SmartDashboard.putNumber("Elevator D", ElevatorConstants.kElevatorD);
    }

    @Override
    protected double getMeasurement() {
        return m_encoder.getPosition();
    }
    /* 
    public void goUp(){
        if (m_elevatorLevel < 2){
            m_elevatorLevel++;
            setSetpoint();
        }
    }

    public void goDown(){
        if (m_elevatorLevel > 0){
            m_elevatorLevel--;
            setSetpoint();
        }
    }

    public void setLevel(int level){
        m_elevatorLevel = level;
        setSetpoint();
    }
    */
    public void setLevel(int level){
        if (isManual != true){
            m_elevatorLevel = level;
            switch (m_elevatorLevel){
                case 0: 
                    setGoal(0);
                    break;
                case 1: 
                    setGoal(30);
                    break;
                case 2:
                    setGoal(70);
                    break;
            }
            enable();
            SmartDashboard.putNumber("Extensor Level", m_elevatorLevel);
            SmartDashboard.putNumber("Extensor goal", getController().getGoal().position);
        }
    }
     @Override
     public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Elevator measurment", getMeasurement());
         if (isManual){
            double output = 0.3 * (-MathUtil.applyDeadband(RobotContainer.m_XboxCommandsController.getLeftY(), OIConstants.kDriveDeadband));
            double ratedOutput = m_dirLimiter.calculate(output);
            SmartDashboard.putNumber("Elevator Joystick", ratedOutput);
            double forwardSoftLimit = (getMeasurement() - ElevatorConstants.kElevatorMaxHeight)/(50-ElevatorConstants.kElevatorMaxHeight);
            double backwardsSoftLimit = (getMeasurement() - ElevatorConstants.kElevatorMinHeight)/(30 - ElevatorConstants.kElevatorMinHeight);
            if ((forwardSoftLimit < 1) && (ratedOutput > 0)){
                m_masterSparkMax.set((ratedOutput * forwardSoftLimit) + 0.05);
            } else if ((backwardsSoftLimit < 1) && (output < 0)) {
                m_masterSparkMax.set((ratedOutput * backwardsSoftLimit) + 0.05);
            } else {
                m_masterSparkMax.set(ratedOutput + 0.05);
            }
            SmartDashboard.putNumber("Elevator applied output", m_masterSparkMax.getAppliedOutput());
         }
     }

    public void setManual(boolean manual){
        isManual = manual;
        if (isManual = true){
            disable();
        } else {
            enable();
        }
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double totalOutput;
        double feedforward = m_feedforward.calculate(setpoint.velocity);
        totalOutput = feedforward + output;
        /* 
        SmartDashboard.putBoolean("Elevator At Setpoint", getController().atSetpoint());
        SmartDashboard.putNumber("Elevator goal", getController().getGoal().position);
        SmartDashboard.putNumber("Elevator feedforward output", feedforward);
        SmartDashboard.putNumber("Elevator setpoint Position", setpoint.position);
        SmartDashboard.putNumber("Elevator setpoint Velocity", setpoint.velocity);
        SmartDashboard.putNumber("Elevator Applied output", totalOutput);
        SmartDashboard.putNumber("Elevator PID output", output);
        */
        m_masterSparkMax.setVoltage(totalOutput);
    }
}
