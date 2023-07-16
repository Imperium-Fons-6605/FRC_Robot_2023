package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Util.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class FieldSim {
    private final SwerveDrive m_swerveDrive;

  private final Field2d m_field2d = new Field2d();

  private Pose2d[] m_swerveModulePoses = {
          new Pose2d(),
          new Pose2d(),
          new Pose2d(),
          new Pose2d()
  };

  public FieldSim(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }

  private void updateRobotPoses() {
    m_field2d.setRobotPose(m_swerveDrive.getPose());
    if(RobotBase.isSimulation()){
      for (int i = 0; i < DriveConstants.kModuleTranslations.length; i++) {
        Translation2d updatedPositions =
          DriveConstants.kModuleTranslations[i]
                        .rotateBy(m_swerveDrive.getPose().getRotation())
                        .plus(m_swerveDrive.getPose().getTranslation());
        m_swerveModulePoses[i] =
                new Pose2d(
                        updatedPositions,
                        m_swerveDrive
                                .getModules()[i]
                                .getHeadingRotation2d()
                                .plus(m_swerveDrive.getHeadingRotation2d()));
      }
  
      m_field2d.getObject("Swerve Modules").setPoses(m_swerveModulePoses);
    }
  }

  public void periodic() {
    updateRobotPoses();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}
