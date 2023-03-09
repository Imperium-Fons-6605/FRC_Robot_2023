package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.ExtensorConstants;
import frc.robot.Util.Constants.OIConstants;

public class ExtensorSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax m_extensorMotor = new CANSparkMax(ExtensorConstants.kExtensorCANId, MotorType.kBrushless);
    private final RelativeEncoder m_extensorEncoder = m_extensorMotor.getEncoder();
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
        ExtensorConstants.kExtensorkSVolts, 
        ExtensorConstants.kExtensorVVoltSecperCm);

    private boolean isManual = false;
    private int m_extensorLevel = 0;


    public ExtensorSubsystem() {
        super(new ProfiledPIDController(
            ExtensorConstants.kExtensorP, 
            ExtensorConstants.kExtensorI, 
            ExtensorConstants.kExtensorD,
            new TrapezoidProfile.Constraints(
                ExtensorConstants.kMaxVelocityCmperSec,
                ExtensorConstants.kMaxAcceleratioCmperSecSquared)
            )
        );
        m_extensorMotor.restoreFactoryDefaults();

        m_extensorMotor.setIdleMode(IdleMode.kBrake);
        m_extensorMotor.setSmartCurrentLimit(50);
        m_extensorEncoder.setPositionConversionFactor(ExtensorConstants.kEncoderPositionFactor);
        m_extensorEncoder.setVelocityConversionFactor(ExtensorConstants.kEncoderVelocityFactor);
        m_extensorEncoder.setPosition(0);
        m_extensorMotor.setInverted(true);

        m_extensorMotor.burnFlash();

        getController().setTolerance(2);
    }

    @Override
    protected double getMeasurement() {
        return m_extensorEncoder.getPosition();
    }
    
    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.velocity);
        double totalOutput = feedforward + output;
        SmartDashboard.putBoolean("Extensor At Setpoint", getController().atGoal());
        SmartDashboard.putNumber("Extensor goal", getController().getGoal().position);
        SmartDashboard.putNumber("Extensor velocity", setpoint.velocity);
        SmartDashboard.putNumber("PID Output", output);
    m_extensorMotor.setVoltage(totalOutput);
    }

    public void setExtension(int level){
        if (isManual != true){
            m_extensorLevel = level;
            switch (m_extensorLevel){
                case 0: 
                    setGoal(0);
                    break;
                case 1: 
                    setGoal(15);
                    break;
                case 2:
                    setGoal(37);
                    break;
            }
        }
        enable();
        SmartDashboard.putNumber("Extensor Level", m_extensorLevel);
    }
     @Override
     public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Extensor measurment", getMeasurement());
        if (isManual){
            double output = 0.15 * (-MathUtil.applyDeadband(RobotContainer.m_GenericCommandsController.getRawAxis(OIConstants.kLogitechRightXAxis), OIConstants.kDriveDeadband));
            double forwardSoftLimit = (getMeasurement() - ExtensorConstants.kExtensorMaxExtension)/(20 - ExtensorConstants.kExtensorMaxExtension);
            double backwardsSoftLimit = (getMeasurement() - ExtensorConstants.kExtensorMinExtension)/(15 - ExtensorConstants.kExtensorMinExtension);
            if ((forwardSoftLimit < 1) && (output > 0)){
                m_extensorMotor.set((output * forwardSoftLimit) + (ExtensorConstants.kExtensorkSPercent * Math.signum(output)));
            } else if ((backwardsSoftLimit < 1) && (output < 0)) {
                m_extensorMotor.set((output * backwardsSoftLimit) + (ExtensorConstants.kExtensorkSPercent * Math.signum(output)));
            } else {
                m_extensorMotor.set(output + (ExtensorConstants.kExtensorkSPercent * Math.signum(output)));
            }
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
}
