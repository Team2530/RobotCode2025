package frc.robot.util;

import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PoseConstants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class LimelightAssistance{
    public static boolean isTargOnLeft(SwerveDrivePoseEstimator odometry){
        int nearestTag = RobotContainer.LLContainer.findNearestTagPos(odometry);
        if(Math.abs(PoseConstants.tagPoses.get(nearestTag).getRotation().getDegrees() - odometry.getEstimatedPosition().getRotation().getDegrees()) > 180){
            return true;
        }
        else{
            return false;
        }
    }
}