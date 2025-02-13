package frc.robot.util;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.Constants.PathPlannerConstants.Pathfinding;

public class PathfindingHelpers {
    // TODO: change to 2025 layout when released of whenever
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    // counterclockwise, starting from the left side of 6 oclock
    public static enum ReefPresets {        
        ALPHA(18, 7, Pathfinding.FACING_LEFT_ADJUST),
        BRAVO(18, 7, Pathfinding.FACING_RIGHT_ADJUST),
        CHARLIE(17, 8, Pathfinding.FACING_LEFT_ADJUST),
        DELTA(17, 8, Pathfinding.FACING_RIGHT_ADJUST),
        ECHO(22, 9, Pathfinding.FACING_LEFT_ADJUST),
        FOXTROT(22, 9, Pathfinding.FACING_RIGHT_ADJUST),
        GOLF(21, 10, Pathfinding.FACING_LEFT_ADJUST),
        HOTEL(21, 10, Pathfinding.FACING_RIGHT_ADJUST),
        INDIA(20, 11, Pathfinding.FACING_LEFT_ADJUST),
        JULIETTE(20, 11, Pathfinding.FACING_RIGHT_ADJUST),
        KILO(19, 6, Pathfinding.FACING_LEFT_ADJUST),
        LIMA(19, 6, Pathfinding.FACING_RIGHT_ADJUST);
        
        private int targetApriltag; 
        private double parallelAdjustment;

        private ReefPresets (int blueSide, int redSide, double parallelAdjustment) {
            if (FieldConstants.getAlliance() == Alliance.Blue) {
                targetApriltag = blueSide;
            } else {
                targetApriltag = redSide;
            }

            this.parallelAdjustment = parallelAdjustment;
        }
    }

    // counterclockwise, starting from 6 oclock
    public static enum ReefFace {
        ONE(ReefPresets.ALPHA, ReefPresets.BRAVO),
        TWO(ReefPresets.CHARLIE, ReefPresets.DELTA),
        THREE(ReefPresets.ECHO, ReefPresets.FOXTROT),
        FOUR(ReefPresets.GOLF, ReefPresets.HOTEL),
        FIVE(ReefPresets.INDIA, ReefPresets.JULIETTE),
        SIX(ReefPresets.KILO, ReefPresets.LIMA);


        private ReefPresets Left;
        private ReefPresets Right;

        private ReefFace(ReefPresets Left, ReefPresets Right) {
            this.Left = Left;
            this.Right = Right;
        }
    }

    public Command generatePathToReefCommand(ReefPresets target) {
        Optional<Pose3d> apriltag = fieldLayout.getTagPose(target.targetApriltag);

        if (apriltag.isPresent()) {
            Pose2d apriltagPose = apriltag.get().toPose2d();
            Rotation2d apriltagDirection = apriltagPose.getRotation();

            // get position by offsetting from the apriltag pose by x in meters, in direction the apriltag is facing
            // then adjust x in meters parallel to the face for left / right pole
            Pose2d targetPose = apriltagPose.plus( 
                new Transform2d(
                    new Translation2d(0.5, apriltagDirection),
                    new Rotation2d()
                )    
            ).plus(
                new Transform2d(
                    new Translation2d(
                        target.parallelAdjustment,
                        apriltagDirection.plus(new Rotation2d(90))
                    ),
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