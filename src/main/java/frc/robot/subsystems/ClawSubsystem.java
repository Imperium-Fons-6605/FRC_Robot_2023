package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.ClawConstants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.OIConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends ProfiledPIDSubsystem{
    private final CANSparkMax m_angleMotor = new CANSparkMax(ClawConstants.kClawAngleCANId, MotorType.kBrushless);
    private final CANSparkMax m_expulsorMotor = new CANSparkMax(ClawConstants.kClawExpulsorCANId, MotorType.kBrushless);
    private final RelativeEncoder m_ExpulsorEncoder = m_expulsorMotor.getEncoder();
    //private final RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
        ClawConstants.kClawSVolts, 
        ClawConstants.kClawGVolts,
        ClawConstants.kClawVVoltSecPerRad);
    
    private int m_clawLevel = 0;
    private double m_grabPercentOutput = 0.4;
    private double m_putPercentOutput = 1;
    private int m_expulseDirection = 1;
    private double m_midLevel = 0.8;
    private double m_highLevel = 1;
    private double m_grabLevel = -0.18;
    private double m_portalLevel = 0.8;

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
        m_angleMotor.restoreFactoryDefaults();
        m_angleMotor.setInverted(false);
        m_angleMotor.setIdleMode(IdleMode.kBrake);
        m_angleMotor.setSmartCurrentLimit(20);
        m_angleMotor.burnFlash();
        m_angleEncoder.setDistancePerRotation(-ClawConstants.kWristEncoderPositionFactor);
        m_expulsorMotor.setInverted(true);
        m_ExpulsorEncoder.setVelocityConversionFactor(ClawConstants.kExpulsorEncoderVelocityFactor);
        getController().setTolerance(0.02);
        getController().enableContinuousInput(-Math.PI, Math.PI);

        if(RobotBase.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(m_angleMotor, DCMotor.getNeo550(1));
        }
    }

    @Override
    protected double getMeasurement() {
        //return m_angleEncoder.getAbsolutePosition();
        return m_angleEncoder.getDistance();
    }
    public double getPosition(){
        return m_angleEncoder.getDistance();
    }
    public double getVelocity(){
        return m_angleEncoder.get() * ClawConstants.kClawMaxVelocityRadPerSec;
    }
    public DutyCycleEncoder getAngleEncoder(){
        return m_angleEncoder;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Claw measurment", getMeasurement());
         if (OIConstants.isManual){
            double output =  0.5 * (MathUtil.applyDeadband(RobotContainer.m_XboxCommandsController.getRightX(), OIConstants.kDriveDeadband));
            double currentMeasurment = getMeasurement();
            double forwardSoftLimit = (currentMeasurment - ClawConstants.kWristMaxRotationRad)/(1-ClawConstants.kWristMaxRotationRad);
            double backwardsSoftLimit = (currentMeasurment - ClawConstants.kWristMinRotationRad)/(0.8 - ClawConstants.kWristMinRotationRad);
            double kG = 0.1 * Math.cos(currentMeasurment);
            if ((forwardSoftLimit < 1) && (output > 0)){
                m_angleMotor.set((output * forwardSoftLimit) + kG);
            } else if ((backwardsSoftLimit < 1) && (output < 0)) {
                m_angleMotor.set((output * backwardsSoftLimit) + kG);
            } else {
                System.out.println("kG" + kG);
                m_angleMotor.set(output + kG);
            }
            m_angleMotor.set(output);
         }

         /*
         if (m_clawLevel == 0){
            RobotContainer.m_XboxCommandsController.button(9).onTrue(new InstantCommand(() -> setGoal(1)));
            RobotContainer.m_XboxCommandsController.button(0).onTrue(new InstantCommand(() -> setGoal(1)));

         }
         */
    }

    public void setLevel(int level){
        if (OIConstants.isManual != true){
            m_clawLevel = level;
            switch (m_clawLevel){
                case 0: 
                    setGoal(2.55);
                    break;
                case 1: 
                    setGoal(m_midLevel);
                    break;
                case 2:
                    setGoal(m_highLevel);
                    break;
                case 3:
                    setGoal(m_portalLevel);
                    break;
                case 4:
                    setGoal(m_grabLevel);
                    break;

            }
            enable();
        }
    }


    @Override
    protected void useOutput(double output, State setpoint) {
        if (!OIConstants.isManual){
            double feedforward = m_feedforward.calculate( getMeasurement(),setpoint.velocity);
            double totalOutput = (feedforward + output);
            m_angleMotor.setVoltage(totalOutput);
        }
    }

     public void setIsCargoCube(boolean pIsCargoCube){
        if (pIsCargoCube){
            m_midLevel = 0.8;
            m_highLevel = 0.7;
            m_grabPercentOutput = 0.6;
            m_expulseDirection = 1;
            m_grabLevel = -0.1;
            m_portalLevel = 1.2;
        } else{                                                                                                                                                                                                                                                                                                                                                                                                     
            m_midLevel= 0.2;
            m_highLevel = 0.4;
            m_grabPercentOutput = 0.6;
            m_expulseDirection = -1;
            m_grabLevel = -0.05;
            m_portalLevel = 0.37;
        }
    }

    public void grabCargo(){
        m_expulsorMotor.set(m_grabPercentOutput * m_expulseDirection);
    }
    public void putCargo(){
        m_expulsorMotor.set(-m_putPercentOutput * m_expulseDirection);
    }
    public void stopClaw(){
        m_expulsorMotor.set(0);
    }
}
