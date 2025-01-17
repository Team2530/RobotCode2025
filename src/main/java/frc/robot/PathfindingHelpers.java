package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;

public class PathfindingHelpers {
    // TODO: change to 2025 layout when released of whenever
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public enum ReefPresets {
        ALPHA(18, 7),
        BRAVO(18, 7),
        CHARLIE(17, 8),
        DELTA(17, 8),
        ECHO(22, 9),
        FOXTROT(22, 9),
        GOLF(21, 10),
        HOTEL(21, 10),
        INDIA(20, 11),
        JULIETTE(20, 11),
        KILO(19, 6),
        LIMA(19, 6);
        
        private int targetApriltag; 

        private ReefPresets (int blueSide, int redSide) {
            if (FieldConstants.getAlliance() == Alliance.Blue) {
                targetApriltag = blueSide;
            } else {
                targetApriltag = redSide;
            }
        }
    }

    public Command generatePathToReefCommand(ReefPresets target) {
        Optional<Pose3d> apriltag = fieldLayout.getTagPose(target.targetApriltag);

        if (apriltag.isPresent()) {
            Pose2d apriltagPose = apriltag.get().toPose2d();

            // offset by x in meters, in direction the apriltag is facing
            Pose2d targetPose = apriltagPose.plus( 
                new Transform2d(
                    new Translation2d(0.5, apriltagPose.getRotation()),
                    new Rotation2d()
                )    
            );
            
            return AutoBuilder.pathfindToPose(
                targetPose, 
                PathPlannerConstants.Pathfinding.CONSTRAINTS,
                0.0
            );
        } else {
            return new Command() {}; // ngl i have no idea if this is valid
        }
        

    }

}