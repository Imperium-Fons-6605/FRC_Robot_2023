package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends ProfiledPIDSubsystem{
    private final CANSparkMax m_angleSparkMax = new CANSparkMax(ClawConstants.kClawAngleCANId, MotorType.kBrushless);
    private final CANSparkMax m_ExpulsorSparkMax = new CANSparkMax(ClawConstants.kClawExpulsorCANId, MotorType.kBrushless);
    private final RelativeEncoder m_ExpulsorEncoder = m_ExpulsorSparkMax.getEncoder();
    //private final RelativeEncoder m_angleEncoder = m_angleSparkMax.getEncoder();
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
        m_angleSparkMax.restoreFactoryDefaults();
        m_angleSparkMax.setInverted(false);
        m_angleSparkMax.setIdleMode(IdleMode.kBrake);
        m_angleSparkMax.setSmartCurrentLimit(20);
        m_angleSparkMax.burnFlash();
        m_angleEncoder.setDistancePerRotation(-ClawConstants.kWristEncoderPositionFactor);
        m_ExpulsorSparkMax.setInverted(true);
        m_ExpulsorEncoder.setVelocityConversionFactor(ClawConstants.kExpulsorEncoderVelocityFactor);
        getController().setTolerance(0.02);
        getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    protected double getMeasurement() {
        //return m_angleEncoder.getAbsolutePosition();
        return m_angleEncoder.getDistance();
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("ClawAppliedOutput", m_ExpulsorSparkMax.getAppliedOutput());
        SmartDashboard.putNumber("Expulsor Encoder Velocity", m_ExpulsorEncoder.getVelocity());



        SmartDashboard.putNumber("Claw measurment", getMeasurement());
        SmartDashboard.putNumber("Claw applied output", m_angleSparkMax.getAppliedOutput());
         if (OIConstants.isManual){
            double output =  0.5 * (MathUtil.applyDeadband(RobotContainer.m_XboxCommandsController.getRightX(), OIConstants.kDriveDeadband));
            double currentMeasurment = getMeasurement();
            double forwardSoftLimit = (currentMeasurment - ClawConstants.kWristMaxRotationRad)/(1-ClawConstants.kWristMaxRotationRad);
            double backwardsSoftLimit = (currentMeasurment - ClawConstants.kWristMinRotationRad)/(0.8 - ClawConstants.kWristMinRotationRad);
            double kG = 0.1 * Math.cos(currentMeasurment);
            if ((forwardSoftLimit < 1) && (output > 0)){
                m_angleSparkMax.set((output * forwardSoftLimit) + kG);
            } else if ((backwardsSoftLimit < 1) && (output < 0)) {
                m_angleSparkMax.set((output * backwardsSoftLimit) + kG);
            } else {
                System.out.println("kG" + kG);
                m_angleSparkMax.set(output + kG);
            }
            m_angleSparkMax.set(output);
            SmartDashboard.putNumber("Claw Joystick", output);
            SmartDashboard.putNumber("Claw kG", (0.4 * Math.cos(getMeasurement())));
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
            SmartDashboard.putNumber("Elevator level", m_clawLevel);
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
            m_angleSparkMax.setVoltage(totalOutput);
        }
    }

     public void setIsCargoCube(boolean pIsCargoCube){
        if (pIsCargoCube){
            m_midLevel = 0.8;
            m_highLevel = 0.6;
            m_grabPercentOutput = 0.6;
            m_expulseDirection = 1;
            m_grabLevel = -0.15;
            m_portalLevel = 1.2;
        } else{                                                                                                                                                                                                                                                                                                                                                                                                     
            m_midLevel= 0.4;
            m_highLevel = 0.5;
            m_grabPercentOutput = 0.6;
            m_expulseDirection = -1;
            m_grabLevel = 0.03;
            m_portalLevel = 0.3;
        }
    }

    public void grabCargo(){
        m_ExpulsorSparkMax.set(m_grabPercentOutput * m_expulseDirection);
    }
    public void putCargo(){
        m_ExpulsorSparkMax.set(-m_putPercentOutput * m_expulseDirection);
    }
    public void stopClaw(){
        m_ExpulsorSparkMax.set(0);
    }
}
