package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.ClawConstants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.OIConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends ProfiledPIDSubsystem{
    private final CANSparkMax m_angleSparkMax = new CANSparkMax(ClawConstants.kClawAngleCANId, MotorType.kBrushless);
    //private final VictorSPX m_angleVictor = new VictorSPX(ClawConstants.kClawAngleCANId);
    private final VictorSPX m_masterExpulsorVictor = new VictorSPX(ClawConstants.kClawMasterExpulsorCANId);
    private final VictorSPX m_slaveExpulsorVictor = new VictorSPX(ClawConstants.kClawSlaveExpulsorCANId);
    private final RelativeEncoder m_angleEncoder = m_angleSparkMax.getEncoder();
    //private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
        ClawConstants.kClawSVolts, 
        ClawConstants.kClawGVolts,
        ClawConstants.kClawVVoltSecPerRad);
    
    private int m_clawLevel = 0;

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
        //m_angleEncoder.setDistancePerRotation(ClawConstants.kWristEncoderPositionFactor);
        //m_angleEncoder.setPositionOffset(m_clawLevel);
        m_angleEncoder.setPositionConversionFactor(ClawConstants.kWristEncoderPositionFactor);
        m_angleEncoder.setPosition(0);
        m_masterExpulsorVictor.setInverted(true);
        m_slaveExpulsorVictor.setInverted(false);
        m_slaveExpulsorVictor.follow(m_masterExpulsorVictor,FollowerType.PercentOutput);
        getController().setTolerance(0.08);
        getController().enableContinuousInput(-Math.PI, Math.PI);
        enable();
    }

    @Override
    protected double getMeasurement() {
        return m_angleEncoder.getPosition() + ClawConstants.kWristEncoderPositionOffset;
        //return m_angleEncoder.getDistance();
    }

    @Override
    public void periodic() {
        super.periodic();


        SmartDashboard.putNumber("Claw measurment", getMeasurement());
        SmartDashboard.putNumber("Claw Applied output", m_angleSparkMax.getAppliedOutput());
         if (OIConstants.isManual){
            double output =  (-MathUtil.applyDeadband(RobotContainer.m_XboxCommandsController.getRightX(), OIConstants.kDriveDeadband));
            double forwardSoftLimit = (getMeasurement() - ClawConstants.kWristMaxRotationRad)/(1-ClawConstants.kWristMaxRotationRad);
            double backwardsSoftLimit = (getMeasurement() - ClawConstants.kWristMinRotationRad)/(0.8 - ClawConstants.kWristMinRotationRad);
            /* 
            if ((forwardSoftLimit < 1) && (output > 0)){
                m_angleSparkMax.set((output * forwardSoftLimit));
            } else if ((backwardsSoftLimit < 1) && (output < 0)) {
                m_angleSparkMax.set((output * backwardsSoftLimit));
            } else {
                m_angleSparkMax.set(output);
            }
            */
            m_angleSparkMax.set(output);
            SmartDashboard.putNumber("Claw Joystick", output);
            SmartDashboard.putNumber("Claw kG", (0.35 * Math.cos(getMeasurement())));
         }
    }

    public void setLevel(int level){
        if (OIConstants.isManual != true){
            m_clawLevel = level;
            SmartDashboard.putNumber("Elevator level", m_clawLevel);
            switch (m_clawLevel){
                case 0: 
                    setGoal(-0.3);
                    break;
                case 1: 
                    setGoal(0);
                    break;
                case 2:
                    setGoal(1);
                    break;
            }
            enable();
        }
    }

    public void setManual(boolean manual){
        OIConstants.isManual = manual;
        if (OIConstants.isManual = true){
            disable();
        } else {
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

    public void setClawPercentOutput(double percentOutput){
        m_masterExpulsorVictor.set(ControlMode.PercentOutput, percentOutput);
    }    
}
