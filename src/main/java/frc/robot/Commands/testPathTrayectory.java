package frc.robot.Commands;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.RobotContainer;
import frc.robot.Util.Constants.AutoConstants;
import frc.robot.Util.Constants.DriveConstants;

public class testPathTrayectory extends SwerveAutoBuilder{

    public testPathTrayectory() {
        super(RobotContainer.m_driveSubsystem::getPose,
        RobotContainer.m_driveSubsystem::resetOdometry, 
        DriveConstants.kDriveKinematics, 
        new PIDConstants(4.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(2, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        RobotContainer.m_driveSubsystem::setModuleStates, 
        AutoConstants.kEventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        RobotContainer.m_driveSubsystem );
        //TODO Auto-generated constructor stub
    }
    
}
