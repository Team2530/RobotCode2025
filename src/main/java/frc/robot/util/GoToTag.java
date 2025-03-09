package frc.robot.util;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.controller.HolonomicDriveController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class GoToTag { //todo -> just put this in swervesubsystem, it was only made a separate class to make writing it less complicated
    final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private SwerveSubsystem swerveSub = null;
    
    TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);
    HolonomicDriveController a = new HolonomicDriveController(null, null, null);
    HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(6.28, 3.14)));


    public GoToTag(SwerveSubsystem swerveSub){
        this.swerveSub = swerveSub;
    };

    void goToTag(int tagToGo){ //TODO: MAKE IT PERIODIC OR ELSE IT WON'T WORK
        Pose3d thePose = tagLayout.getTagPose(tagToGo).get(); //TODO: ADJUST THIS TO BE AWAY FROM THE THING, NOT AT IT
        Pose2d thePose2d = new Pose2d(new Translation2d(thePose.getX(), thePose.getY()), thePose.getRotation().toRotation2d());
        Pose2d currPose = swerveSub.odometry.getEstimatedPosition();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            currPose, 
            List.of(),  // No intermediate waypoints (just a straight line)
            thePose2d, 
            config
        );
        Trajectory.State goal = trajectory.sample(timeSinceStart); //todo -> find time since tag was made.
        ChassisSpeeds adjustedSpeeds = controller.calculate(
            currPose, goal, Rotation2d.fromDegrees(0));
        SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(adjustedSpeeds);
        swerveSub.setModules(moduleStates);
    }

    
}
