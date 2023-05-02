package frc.robot.simulation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Util.Constants.ClawConstants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ExtensorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensorSubsystem;

public class MechanismSim {

    private final Mechanism2d m_mech2d = new Mechanism2d(70, 110);
    private final MechanismRoot2d m_Root2d = m_mech2d.getRoot("MechRoot", 35, 5);
    
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
        DCMotor.getNEO(2),
        ElevatorConstants.kElevatorMotorReduction,
        ElevatorConstants.kCarriageMassKg,
        1,
        ElevatorConstants.kElevatorMinHeightCm,
        ElevatorConstants.kElevatorMaxHeightCm,
        true,
        VecBuilder.fill(0.01)
      );
      private final ElevatorSim m_extensorSim = new ElevatorSim(
        DCMotor.getNEO(1),
        ExtensorConstants.kElevatorMotorReduction,
        ExtensorConstants.kCarriageMassKg,
        1,
        ExtensorConstants.kExtensorMinExtension,
        ExtensorConstants.kExtensorMaxExtension,
        false,
        VecBuilder.fill(0.01)
      );
    private final SingleJointedArmSim m_clawSim = new SingleJointedArmSim(
      DCMotor.getNeo550(1),
       ClawConstants.kWristMotorReduction,
        0.3,
         0.5,
          ClawConstants.kWristMinRotationRad,
           ClawConstants.kWristMaxRotationRad,
            5,
             true);
    private final MechanismLigament2d m_elevatorMech2d;
    private final MechanismLigament2d m_extensorMech2d;
    private final MechanismLigament2d m_clawMech2d;

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ExtensorSubsystem m_extensorSubsystem;
    private final ClawSubsystem m_clawSubsystem;

    
    private final DutyCycleEncoderSim m_angleEncoderSim;

    public MechanismSim(ElevatorSubsystem m_elevatorSubsystemP, ExtensorSubsystem m_extensorSubsystemP, ClawSubsystem m_clawSubsystemP){
        m_elevatorSubsystem = m_elevatorSubsystemP;
        m_extensorSubsystem = m_extensorSubsystemP;
        m_clawSubsystem = m_clawSubsystemP;

        m_elevatorMech2d = new MechanismLigament2d("Elevator2d", m_elevatorSubsystem.getPosition(), 90);
        m_extensorMech2d = new MechanismLigament2d("Extensor2d", m_extensorSubsystem.getPosition(), 90);
        m_clawMech2d = new MechanismLigament2d("Claw2d", 20, m_clawSubsystem.getPosition());

        m_extensorMech2d.setColor(new Color8Bit(Color.kCyan));
        m_clawMech2d.setColor(new Color8Bit(Color.kGreen));

        m_Root2d.append(m_elevatorMech2d);
        m_elevatorMech2d.append(m_extensorMech2d);
        m_extensorMech2d.append(m_clawMech2d);

        m_angleEncoderSim = new DutyCycleEncoderSim(m_clawSubsystem.getAngleEncoder());
        SmartDashboard.putData("Robot2d", m_mech2d);
    }

    public void periodic(){
      m_elevatorMech2d.setLength(m_elevatorSubsystem.getPosition());
      m_extensorMech2d.setLength(m_extensorSubsystem.getPosition());
      m_clawMech2d.setAngle(m_clawSubsystem.getPosition());
      m_angleEncoderSim.setDistance(m_clawSim.getAngleRads());

    }
    public void simulationPeriodic() {
      m_elevatorSim.setInput(m_elevatorSubsystem.getVelocity());
      m_elevatorSim.update(0.020);

      m_extensorSim.setInput(m_extensorSubsystem.getVelocity());
      m_extensorSim.update(0.020);

      m_clawSim.setInput(m_clawSubsystem.getVelocity());
      m_clawSim.update(0.020);
    }
}
