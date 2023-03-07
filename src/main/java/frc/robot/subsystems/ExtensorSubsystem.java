package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.ExtensorConstants;
import frc.robot.Util.Constants.OIConstants;

public class ExtensorSubsystem extends PIDSubsystem {
    private final CANSparkMax m_extensorMotor = new CANSparkMax(ExtensorConstants.kExtensorCANId, MotorType.kBrushless);
    private final RelativeEncoder m_extensorEncoder = m_extensorMotor.getEncoder();

    private boolean isManual = false;
    private int m_extensorLevel = 0;


    public ExtensorSubsystem() {
        super(new PIDController(
            ExtensorConstants.kExtensorP, 
            ExtensorConstants.kExtensorI, 
            ExtensorConstants.kExtensorD
            )
        );
        m_extensorMotor.restoreFactoryDefaults();

        m_extensorMotor.setIdleMode(IdleMode.kBrake);
        m_extensorMotor.setSmartCurrentLimit(50);
        m_extensorEncoder.setPositionConversionFactor(ExtensorConstants.kEncoderPositionFactor);
        m_extensorEncoder.setVelocityConversionFactor(ExtensorConstants.kEncoderVelocityFactor);
        m_extensorEncoder.setPosition(0);

        m_extensorMotor.burnFlash();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        /* 
        SmartDashboard.putBoolean("At Setpoint", getController().atSetpoint());
        SmartDashboard.putNumber("goal", getController().getSetpoint());
        SmartDashboard.putNumber("???", setpoint);
        SmartDashboard.putNumber("measurment", getMeasurement());
        SmartDashboard.putNumber("PID Output", output);
        */
        if (getMeasurement() < 0){
            m_extensorMotor.set(0);
        } else if (getMeasurement() > 40) {
            m_extensorMotor.set(0);
        } else {
            m_extensorMotor.set(output);
        }
    }

    @Override
    protected double getMeasurement() {
        return m_extensorEncoder.getPosition();
    }

    public void extend(){
        if (m_extensorLevel > 0){
            m_extensorLevel++;
            setSetpoint();
        }
    }
    public void retract(){
        if (m_extensorLevel < 2){
            m_extensorLevel--;
            setSetpoint();
        }
    }

    public void setLevel(int level){
        m_extensorLevel = level;
        setSetpoint();
    }

    public void setSetpoint(){
        if (isManual != true){
            switch (m_extensorLevel){
                case 0: 
                    setSetpoint(0);
                    break;
                case 1: 
                    setSetpoint(15);
                    break;
                case 2:
                    setSetpoint(30);
                    break;
            }
        }
    }
     @Override
     public void periodic() {
         if (isManual){
            double output = MathUtil.applyDeadband(RobotContainer.m_GenericCommandsController.getRawAxis(OIConstants.kLogitechRightXAxis), OIConstants.kDriveDeadband);
            if (getMeasurement() < 0){
                m_extensorMotor.set(0);
            } else if (getMeasurement() > 40) {
                m_extensorMotor.set(0);
            } else {
                if (MathUtil.applyDeadband(output, OIConstants.kDriveDeadband) == 0) {
                    m_extensorMotor.set(output + 0.08 * Math.signum(output));
                }
            }
         }
     }
    public void toggleManual (){
        if (isManual == false){
            isManual = true;
        } else {
            isManual = false;
        }
    }
    
}
