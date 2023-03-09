package frc.robot.Commands;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.DriveConstants;

public class Autos {

    private List<PathPlannerTrajectory> m_oneTrayectory;
    private List<PathPlannerTrajectory> m_twoTrayectory;
    private List<PathPlannerTrajectory> m_threeTrayectory;
    private List<PathPlannerTrajectory> m_oneChargeTrayectory;
    private List<PathPlannerTrajectory> m_twoChargeTrayectory;
    private List<PathPlannerTrajectory> m_threeChargeTrayectory;

    private CommandBase m_oneAuto;
    private CommandBase m_twoAuto;
    private CommandBase m_threeAuto;
    private CommandBase m_oneChargeAuto;
    private CommandBase m_twoChargeAuto;
    private CommandBase m_threeChargeAuto;

    private SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        RobotContainer.m_driveSubsystem::getPose, 
        RobotContainer.m_driveSubsystem::resetOdometry, 
        DriveConstants.kDriveKinematics, 
        new PIDConstants(3, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        RobotContainer.m_driveSubsystem::setModuleStates, 
        AutoConstants.kEventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        RobotContainer.m_driveSubsystem 
    );

    public Autos(){
        PathPlannerServer.startServer(6605);
    }

    public Command getAuto (int position, boolean isBalance){
        if (m_oneAuto == null){
            System.out.println("No se han inciializado los autos");
            return null;
        } else {
            if(!isBalance){
                switch (position) {
                    case 1:
                        return m_oneAuto;
                    case 2:
                        return m_twoAuto;
                    case 3:
                        return m_threeAuto;
                }
            } else {
                switch (position) {
                    case 1:
                        return m_oneAuto.andThen(m_oneChargeAuto).andThen(Commands.run(() ->RobotContainer.m_driveSubsystem.setX(), (Subsystem)RobotContainer.m_driveSubsystem));
                    case 2:
                        return m_twoAuto.andThen(m_twoChargeAuto).andThen(Commands.run(() ->RobotContainer.m_driveSubsystem.setX(), (Subsystem)RobotContainer.m_driveSubsystem));
                    case 3:
                        return m_threeAuto.andThen(m_threeChargeAuto).andThen(Commands.run(() ->RobotContainer.m_driveSubsystem.setX(), (Subsystem)RobotContainer.m_driveSubsystem));
                }
            }
            return new InstantCommand(() -> RobotContainer.m_driveSubsystem.drive(0.2, 0, 0, true, true)).withTimeout(3);
        }
    }

    public void buildAutos(){
        m_oneTrayectory = PathPlanner.loadPathGroup("1 auto", AutoConstants.kAutoConstraints);
        m_twoTrayectory = PathPlanner.loadPathGroup("2 auto", AutoConstants.kAutoConstraints);
        m_threeTrayectory = PathPlanner.loadPathGroup("3 auto", AutoConstants.kAutoConstraints);

        m_oneChargeTrayectory = PathPlanner.loadPathGroup("1 charge auto", AutoConstants.kAutoConstraints);
        m_twoChargeTrayectory = PathPlanner.loadPathGroup("2 charge auto", AutoConstants.kAutoConstraints);
        m_threeChargeTrayectory = PathPlanner.loadPathGroup("3 charge auto", AutoConstants.kAutoConstraints);

        m_oneAuto =  autoBuilder.fullAuto(m_oneTrayectory);
        m_twoAuto =  autoBuilder.fullAuto(m_twoTrayectory);
        m_threeAuto =  autoBuilder.fullAuto(m_threeTrayectory);

        m_oneChargeAuto =  autoBuilder.fullAuto(m_oneChargeTrayectory);
        m_twoChargeAuto =  autoBuilder.fullAuto(m_twoChargeTrayectory);
        m_threeChargeAuto =  autoBuilder.fullAuto(m_threeChargeTrayectory);
    }
    
}
