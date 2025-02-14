package frc.robot.util;

import frc.robot.RobotContainer;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class LimelightAssistance{

    SwerveSubsystem swerveSub = null;
    public LimelightAssistance(SwerveSubsystem swerveSub){
        this.swerveSub = swerveSub;
    }

    public boolean isTagOnLeft(){

        int nearestTag = RobotContainer.LLContainer.findNearestTagPos(swerveSub.odometry);
        if(Math.abs(PoseConstants.tagPoses.get(nearestTag).getRotation().getDegrees() - swerveSub.odometry.getEstimatedPosition().getRotation().getDegrees()) > 180){
            return true;
        }
        else{
            return false;
        }
    }
}