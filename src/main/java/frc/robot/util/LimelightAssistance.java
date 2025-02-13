package frc.robot.util;

import frc.robot.util.RobotContainer;

public class LimelightAssistance{
    public static boolean isTargOnLeft(SwerveDrivePoseEstimator odometry){
        Pose2d tagPose = RobotContainer.LLContainer.getNearestTagPos();
        Pose2d botPose = odometry.getBotPoseEstimate();
        if(Math.abs(tagPose.getRotation2d() - botPose.getRotation2d()) > 180){
            return true;
        }
        else{
            return false;
        }
    }
}