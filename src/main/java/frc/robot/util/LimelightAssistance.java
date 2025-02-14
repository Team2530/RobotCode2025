package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.coral.CoralSubsystem.MirrorPresets;


public class LimelightAssistance{

    public MirrorPresets optimalMirrorToTag(int tag, Pose2d robotPose){

        if(Math.abs(PoseConstants.tagPoses.get(tag).getRotation().getDegrees() - robotPose.getRotation().getDegrees()) > 180){
            return MirrorPresets.PORT;
        }
        else{
            return MirrorPresets.STARBOARD;
        }
    }
}