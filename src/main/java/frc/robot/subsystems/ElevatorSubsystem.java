package frc.robot.subsystems;


import org.ietf.jgss.GSSException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation3d;
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
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        ElevatorConstants.kElevatorSVolts,
        ElevatorConstants.kElevatorGVolts,
        ElevatorConstants.kElevatorVVoltSecperMeter);
    private final RelativeEncoder m_encoder = m_masterSparkMax.getEncoder();

    private boolean isManual = false;
    private int m_elevatorLevel = 0;

    private double currentkP = ElevatorConstants.kElevatorP;
    private double currentkD = ElevatorConstants.kElevatorD;

    public ElevatorSubsystem() {
        super(new ProfiledPIDController(
            ElevatorConstants.kElevatorP,
            ElevatorConstants.kElevatorI,
            ElevatorConstants.kElevatorD, 
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxVelocityMeterperSec, 
                ElevatorConstants.kMaxAcceleratioMeterperSecSquared)
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

        SmartDashboard.putData(new InstantCommand(() -> m_encoder.setPosition(0)));
        SmartDashboard.putNumber("Elevator P", ElevatorConstants.kElevatorP);
        SmartDashboard.putNumber("Elevator D", ElevatorConstants.kElevatorD);
    }

    @Override
    protected double getMeasurement() {
        return m_encoder.getPosition();
    }
    
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

    public void setSetpoint(){
        System.out.println("Aplicando setpoint sin aplicar...");
        if (isManual != true){
            System.out.println("Aplicando nuevo setpoint...");
            SmartDashboard.putNumber("Elevator level", m_elevatorLevel);
            switch (m_elevatorLevel){
                case 0: 
                    setGoal(0);
                    break;
                case 1: 
                    setGoal(30);
                    break;
                case 2:
                    setGoal(80);
                    break;
            }
            enable();
        }
    }
     @Override
     public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("measurment", getMeasurement());
        double kP = SmartDashboard.getNumber("Elevator P", ElevatorConstants.kElevatorP);
        double kD = SmartDashboard.getNumber("Elevator D", ElevatorConstants.kElevatorD);
        if (kP != currentkP) {
            currentkP = kP;
            getController().setP(currentkP);
        }
        if (kD != currentkD) {
            currentkP = kD;
            getController().setD(currentkD);
        }
         if (isManual){
            double output = -MathUtil.applyDeadband(RobotContainer.m_GenericCommandsController.getRawAxis(OIConstants.kLogitechLeftYAxis), OIConstants.kDriveDeadband);
            if (getMeasurement() < 0){
                m_masterSparkMax.set( -(getMeasurement() - 0 * 0.007));
            } else if (getMeasurement() > 80) {
                m_masterSparkMax.set(-(getMeasurement() - 80 * 0.007));
            } else {
                m_masterSparkMax.set(output + 0.08 * Math.signum(output) + 0.02);
            }
         }
     }

    public void toggleManual (){
        if (isManual == false){
            isManual = true;
            disable();
        } else {
            isManual = false;
            enable();
        }
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double totalOutput;
        double feedforward = m_feedforward.calculate(setpoint.velocity);
        //double pidOutput = (getMeasurement() - getController().getGoal().position) * 0.02;
        //totalOutput =  Math.min(-1, Math.max(1,(feedforward + ( pidOutput))));
        totalOutput = feedforward + output;
        SmartDashboard.putData("ProfiledPIDControler", getController());
        SmartDashboard.putNumber("Current Profiled Setpoint to reach", setpoint.position);
        SmartDashboard.putBoolean("Elevator At Setpoint", getController().atSetpoint());
        SmartDashboard.putNumber("Elevator goal", getController().getGoal().position);
        SmartDashboard.putNumber("Elevator feedforward output", feedforward);
        SmartDashboard.putNumber("Elevator setpoint Position", setpoint.position);
        SmartDashboard.putNumber("Elevator setpoint Velocity", setpoint.velocity);
        SmartDashboard.putNumber("Elevator Applied output", totalOutput);
        m_masterSparkMax.setVoltage(totalOutput);
        /* 
        if (getMeasurement() < 0){
            m_masterSparkMax.set( -(getMeasurement() - 0 * 0.02));
        } else if (getMeasurement() > 80) {
            m_masterSparkMax.set(-(getMeasurement() - 80 * 0.02));
        } else {
            m_masterSparkMax.setVoltage(totalOutput);
        }
        */
    }
}
