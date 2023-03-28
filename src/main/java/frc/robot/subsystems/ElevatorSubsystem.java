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
import frc.robot.Util.Constants.ClawConstants;
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

    private int m_elevatorLevel = 0;
    private int m_midLevel = 45;
    private int m_portalLevel = 0;

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
        enable();
    }

    @Override
    protected double getMeasurement() {
        return m_encoder.getPosition();
    }
    public void setLevel(int level){
        if (OIConstants.isManual != true){
            m_elevatorLevel = level;
            switch (m_elevatorLevel){
                case 0: 
                    setGoal(-1);
                    break;
                case 1: 
                    setGoal(m_midLevel);
                    break;
                case 2:
                    setGoal(80);
                    break;
                case 3:
                    setGoal(m_portalLevel);
                    break;
            }
            enable();
        }
    }
     @Override
     public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Elevator measurment", getMeasurement());
         if (OIConstants.isManual){
            double output = 0.4 * (-MathUtil.applyDeadband(RobotContainer.m_XboxCommandsController.getLeftY(), OIConstants.kDriveDeadband));
            double ratedOutput = m_dirLimiter.calculate(output);
            double forwardSoftLimit = (getMeasurement() - ElevatorConstants.kElevatorMaxHeight)/(70-ElevatorConstants.kElevatorMaxHeight);
            double backwardsSoftLimit = (getMeasurement() - ElevatorConstants.kElevatorMinHeight)/(15 - ElevatorConstants.kElevatorMinHeight);
            if ((forwardSoftLimit < 1) && (ratedOutput > 0)){
                m_masterSparkMax.set((ratedOutput * forwardSoftLimit) + 0.03);
            } else if ((backwardsSoftLimit < 1) && (output < 0)) {
                m_masterSparkMax.set((ratedOutput * backwardsSoftLimit)+ 0.03);
            } else {
                m_masterSparkMax.set(ratedOutput + 0.03);
            }
         }
     }

    public void setIsCargoCube(boolean pIsCargoCube){
        if (pIsCargoCube){
            m_midLevel = 45;
            m_portalLevel = 0;
            LEDSubsystem.getInstance().setCustomColor(255, 255, 255);
        } else{
            m_midLevel = 60;
            m_portalLevel = 80;
            LEDSubsystem.getInstance().setCustomColor(150, 255, 255);
        }
    }
    @Override
    protected void useOutput(double output, State setpoint) {
        if (!OIConstants.isManual){
            if(!getController().atGoal()){
                double feedforward = m_feedforward.calculate(setpoint.velocity);
                double totalOutput = output + feedforward;
                m_masterSparkMax.setVoltage(totalOutput);
            } else {
                m_masterSparkMax.setVoltage(ElevatorConstants.kElevatorGVolts);
            }
        }
    }
}
