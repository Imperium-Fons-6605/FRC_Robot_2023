package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

        //m_encoderSim = new EncoderSim((Encoder) m_encoder.);
        
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

        if (RobotBase.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(m_masterSparkMax, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_slaveSparkMax, DCMotor.getNEO(1));
        }
    }

    @Override
    protected double getMeasurement() {
        return m_encoder.getPosition();
    }
    public double getPosition(){
        return m_encoder.getPosition();
    }
    public double getVelocity(){
        return m_encoder.getVelocity();
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
            double forwardSoftLimit = (getMeasurement() - ElevatorConstants.kElevatorMaxHeightCm)/(70-ElevatorConstants.kElevatorMaxHeightCm);
            double backwardsSoftLimit = (getMeasurement() - ElevatorConstants.kElevatorMinHeightCm -12)/(15 - ElevatorConstants.kElevatorMinHeightCm -12);
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
