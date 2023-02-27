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
    EncoderTypes encoderType;

    public enum EncoderTypes{ABSOLUTE, RELATIVE}

    public ImperiumSparkMax(int deviceId, MotorType type, boolean inverted, 
    EncoderTypes encoderTypep, double positionFactor, double velocityFactor,
    double Kp, double Ki, double Kd, double Kff, int ampersLimit) {
        
        
        super(deviceId, type);
        
        restoreFactoryDefaults();
        setInverted(inverted);
        pidController = getPIDController();
        setPIDF(Kp, Ki, Kd, Kff);
        pidController.setOutputRange(-1, 1);

        setIdleMode((type == MotorType.kBrushless) ? IdleMode.kBrake : IdleMode.kCoast);
        setSmartCurrentLimit(ampersLimit);
        encoderType = encoderTypep;

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

    public void follow (ImperiumSparkMax master){
        follow(master);
    }

    public double getPosition(){
        if (encoderType == EncoderTypes.ABSOLUTE){
            return absoluteEncoder.getPosition();
        } else {
            return relativeEncoder.getPosition();
        }
    }

    public double getVelocity(){
        if (encoderType == EncoderTypes.ABSOLUTE){
            return absoluteEncoder.getVelocity();
        } else {
            return relativeEncoder.getVelocity();
        }
    }

    public double getPercentOutput(){
        return getAppliedOutput();
    }

    public double getCPR(){
        if (encoderType == EncoderTypes.RELATIVE){
        return relativeEncoder.getCountsPerRevolution();
        }
        return 0;
    }
    public void setRelativeEncoderPosition(double position){
        if (encoderType == EncoderTypes.RELATIVE){
            relativeEncoder.setPosition(position);
        }
    }

    


}