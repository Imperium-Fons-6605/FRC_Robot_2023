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

public class ClawSubsystem extends ProfiledPIDSubsystem{
    private final VictorSPX m_angleVictor = new VictorSPX(ClawConstants.kClawAngleCANId);
    private final VictorSPX m_masterExpulsorVictor = new VictorSPX(ClawConstants.kClawMasterExpulsorCANId);
    private final VictorSPX m_slaveExpulsorVictor = new VictorSPX(ClawConstants.kClawSlaveExpulsorCANId);
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(ClawConstants.kClawAngleEncoderDIOPort);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
        ClawConstants.kClawSVolts, 
        ClawConstants.kClawGVolts,
        ClawConstants.kClawVVoltSecPerRad);

    private final SlewRateLimiter m_dirLimiter = new SlewRateLimiter(ClawConstants.kDirectionSlewRate);
    
    private boolean isManual = false;
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
        m_angleEncoder.setDistancePerRotation(ClawConstants.kWristEncoderPositionFactor);
        m_angleEncoder.setPositionOffset(ClawConstants.kWristEncoderPositionOffset);
        m_angleVictor.setInverted(true);
        m_masterExpulsorVictor.setInverted(true);
        m_slaveExpulsorVictor.setInverted(false);
        m_slaveExpulsorVictor.follow(m_masterExpulsorVictor,FollowerType.PercentOutput);
        getController().setTolerance(0.08);
        getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    protected double getMeasurement() {
        return m_angleEncoder.getDistance();
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Claw measurment", getMeasurement());
        SmartDashboard.putNumber("Claw Applied output", m_angleVictor.getMotorOutputPercent());
         if (isManual){
            double output = 0.25 * (-MathUtil.applyDeadband(RobotContainer.m_XboxCommandsController.getRightX(), OIConstants.kDriveDeadband));
            double ratedOutput = m_dirLimiter.calculate(output);
            double forwardSoftLimit = (getMeasurement() - ClawConstants.kWristMaxRotationRad)/(1-ClawConstants.kWristMaxRotationRad);
            double backwardsSoftLimit = (getMeasurement() - ClawConstants.kWristMinRotationRad)/(0.8 - ClawConstants.kWristMinRotationRad);
            if ((forwardSoftLimit < 1) && (ratedOutput > 0)){
                m_angleVictor.set(VictorSPXControlMode.PercentOutput,(ratedOutput * forwardSoftLimit) + (0.25 * Math.cos(getMeasurement())));
            } else if ((backwardsSoftLimit < 1) && (output < 0)) {
                m_angleVictor.set(VictorSPXControlMode.PercentOutput,(ratedOutput * backwardsSoftLimit) + (0.25 * Math.cos(getMeasurement())));
            } else {
                m_angleVictor.set(VictorSPXControlMode.PercentOutput,ratedOutput + (0.25 * Math.cos(getMeasurement())));
            }
            SmartDashboard.putNumber("Claw Joystick", ratedOutput);
            SmartDashboard.putNumber("Claw kG", (0.25 * Math.cos(getMeasurement())));
         }
    }

    public void setLevel(int level){
        if (isManual != true){
            m_clawLevel = level;
            SmartDashboard.putNumber("Elevator level", m_clawLevel);
            switch (m_clawLevel){
                case 0: 
                    setGoal(-0.2);
                    break;
                case 1: 
                    setGoal(0);
                    break;
                case 2:
                    setGoal(-0.3);
                    break;
            }
            enable();
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
        double feedforward = m_feedforward.calculate( getMeasurement(),setpoint.velocity);
        double totalOutput = (feedforward + output) / 12;
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
