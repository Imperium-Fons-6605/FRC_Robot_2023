package frc.robot.Util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ImperiumSparkMax extends CANSparkMax {

    private SparkMaxPIDController pidController;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;

    public enum EncoderTypes{ABSOLUTE, RELATIVE}

    public ImperiumSparkMax(int deviceId, MotorType type, boolean inverted, 
    EncoderTypes encoderType, double positionFactor, double velocityFactor,
    double Kp, double Ki, double Kd, double Kff, int ampersLimit) {
        
        super(deviceId, type);
        
        restoreFactoryDefaults();
        setInverted(inverted);
        pidController = getPIDController();
        setPIDF(Kp, Ki, Kd, Kff);
        pidController.setOutputRange(-1, 1);

        setIdleMode((type == MotorType.kBrushless) ? IdleMode.kBrake : IdleMode.kCoast);
        setSmartCurrentLimit(ampersLimit);

        if (encoderType == EncoderTypes.ABSOLUTE){
            absoluteEncoder = getAbsoluteEncoder(Type.kDutyCycle);
            pidController.setFeedbackDevice(absoluteEncoder);
            absoluteEncoder.setPositionConversionFactor(positionFactor);
            absoluteEncoder.setVelocityConversionFactor(velocityFactor);
        } else {
            relativeEncoder = getEncoder();
            pidController.setFeedbackDevice(relativeEncoder);
            relativeEncoder.setPositionConversionFactor(positionFactor);
            relativeEncoder.setVelocityConversionFactor(velocityFactor);
        }
    }

    public void setPIDF(double p, double i, double d, double f){
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(f);
    }


}