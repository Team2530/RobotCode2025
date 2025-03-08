package frc.robot.util;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

public class GoToTag {
    int tagToGo = 0;
    public GoToTag(int tagToGo){}

    static void goToTag(int tagToGo){
        final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        Optional<Pose3d> thePose = tagLayout.getTagPose(tagToGo);
        

    }
}
