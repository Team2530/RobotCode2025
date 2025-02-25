package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.coral.CoralSubsystem.MirrorPresets;
import frc.robot.RobotContainer;


public class LimelightAssistance{

    SwerveSubsystem swerveSub = null;

    public LimelightAssistance(SwerveSubsystem swerveSub){
        this.swerveSub = swerveSub;
    }

    public boolean isTagOnRight(){

        SmartDashboard.putString("Recieved position to isTagOnLeft: ", swerveSub.odometry.getEstimatedPosition().toString());
        int nearestTag = RobotContainer.LLContainer.findNearestTagPos(swerveSub.odometry);
        SmartDashboard.putNumber("Recieved nearest tag to isTagOnLeft: ", nearestTag);
        SmartDashboard.putString("Tag angle, robot angle: ", ""+PoseConstants.tagPoses.get(nearestTag).getRotation().getDegrees() + swerveSub.odometry.getEstimatedPosition().getRotation().getDegrees());
        if(Math.abs(PoseConstants.tagPoses.get(nearestTag).getRotation().getDegrees() - swerveSub.odometry.getEstimatedPosition().getRotation().getDegrees()) > 180){
            SmartDashboard.putBoolean("Tag is on left: ", true);
            return true;
        }
        else{
            SmartDashboard.putBoolean("Tag is on left: ", false);
            return false;
        }
    }
}