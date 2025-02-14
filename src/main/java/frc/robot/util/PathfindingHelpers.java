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
import frc.robot.Constants.Elevator;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.PathPlannerConstants.Pathfinding;
import frc.robot.subsystems.coral.CoralSubsystem.MirrorPresets;

public class PathfindingHelpers {
    // TODO: change to 2025 layout when released of whenever
    private final static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

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
    public static enum ReefFaces {
        ONE(ReefPresets.ALPHA, ReefPresets.BRAVO),
        TWO(ReefPresets.CHARLIE, ReefPresets.DELTA),
        THREE(ReefPresets.ECHO, ReefPresets.FOXTROT),
        FOUR(ReefPresets.GOLF, ReefPresets.HOTEL),
        FIVE(ReefPresets.INDIA, ReefPresets.JULIETTE),
        SIX(ReefPresets.KILO, ReefPresets.LIMA);


        public ReefPresets Left;
        public ReefPresets Right;

        private ReefFaces(ReefPresets Left, ReefPresets Right) {
            this.Left = Left;
            this.Right = Right;
        }
    }

    public static Command generatePathToReefCommand(ReefPresets target, Pose2d startingPose) {
        Optional<Pose3d> apriltag = fieldLayout.getTagPose(target.targetApriltag);

        if (apriltag.isPresent()) {
            Pose2d apriltagPose = apriltag.get().toPose2d();
            Rotation2d apriltagDirection = apriltagPose.getRotation();

            // which side of the robot the reef is closer to
            MirrorPresets optimalMirror = optimalMirrorToReef(startingPose); 

            // get position by offsetting from the apriltag pose by x in meters, in direction the apriltag is facing
            // then adjust x in meters parallel to the face for left / right pole
            // and for coral arm mirroring
            Pose2d targetPose = apriltagPose.plus( 
                new Transform2d(
                    new Translation2d(0.5, apriltagDirection),
                    new Rotation2d()
                )    
            ).plus(
                new Transform2d(
                    new Translation2d( // left / right pole adjustment
                        target.parallelAdjustment
                        + ( // mirror side adjustment
                            Elevator.PhysicalParameters.CORAL_PIVOT_HORIZONTAL_OFFSET
                            * (optimalMirror.isMirrored ? -1 : 1)
                        ),
                        apriltagDirection.plus(new Rotation2d(90))
                    ),
                    // mirror side rotation
                    new Rotation2d(Math.PI * (optimalMirror.isMirrored ? -1 : 1))
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

    public static ReefFaces getNearestReefFace(Pose2d robotPosition) {
        Pose2d relativePosition = robotPosition.relativeTo(FieldConstants.REEF_POSITION);
        // this is terrible
        // get the angle of a line drawn from the center of the reef to the robot,
        // oriented so that an angle of 0 points to reef face ONE, moving 
        // counterclockwise as the angle increases
        // in radians
        double orientedRelativeAngle = (
            (Math.PI * 2) + Math.atan2(relativePosition.getY(), relativePosition.getX()) 
            - FieldConstants.REEF_POSITION.getRotation().getRadians()
        ) % (Math.PI * 2);
        // this is barely better
        return ReefFaces.values()[
            ((int) 
                Math.round(
                    orientedRelativeAngle / (Math.PI / 3)
                )
            ) - 1
        ];
    }

    public static MirrorPresets optimalMirrorToReef(Pose2d robotPosition) {
        Pose2d relativePose = FieldConstants.REEF_POSITION.relativeTo(robotPosition);
        // angle of a line from the robot to the reef, relative to the heading of the robot
        // in radians
        double orientedRelativeAngle = (
            (Math.PI * 2) + Math.atan2(relativePose.getY(), relativePose.getX())
            - robotPosition.getRotation().getRadians()
        ) % (Math.PI * 2);
        return orientedRelativeAngle < Math.PI
            ? MirrorPresets.PORT
            : MirrorPresets.STARBOARD;
    }
}