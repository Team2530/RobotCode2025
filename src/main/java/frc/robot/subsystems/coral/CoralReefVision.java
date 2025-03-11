package frc.robot.subsystems.coral;

import java.util.ArrayList;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Coral.Vision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import frc.robot.util.CoralReefVisionSim;
import frc.robot.util.Reef;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

@Logged(strategy = Strategy.OPT_IN)
public class CoralReefVision extends SubsystemBase {

    // Vision targets in robot space
    private final StructArrayPublisher<Translation3d> visionTargetPublisher;
    // Camera pose in robot space
    private final StructPublisher<Pose3d> cameraPublisher;

    // Debug outputs
    private StructPublisher<Pose3d> primaryVisionTargetFieldSpace;
    private StructArrayPublisher<Pose3d> visionTargetsFieldSpace;
    private StructPublisher<Pose2d> scoringPoseFieldSpace;
    private StructPublisher<Pose3d> cameraPoseFieldSpace;

    // Inputs from vision coprocessor
    private final DoubleArraySubscriber inputAngles;
    private final DoubleArraySubscriber inputDistances;

    @Logged
    private ArrayList<Translation3d> visionTargets = new ArrayList<Translation3d>();
    private int selectedTargetIndex = -1;

    private CoralReefVisionSim sim;

    public CoralReefVision() {
        inputAngles = NetworkTableInstance.getDefault()
                .getDoubleArrayTopic("CoralVision/raw/angles").subscribe(new double[] {});
        inputDistances = NetworkTableInstance.getDefault()
                .getDoubleArrayTopic("CoralVision/raw/distances").subscribe(new double[] {});

        visionTargetPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("CoralVision/targets", Translation3d.struct).publish();
        cameraPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("CoralVision/cameraPose", Pose3d.struct).publish();

        // Debug publishers
        primaryVisionTargetFieldSpace = NetworkTableInstance.getDefault()
                .getStructTopic("CoralVision/selectedTargetPose",
                        Pose3d.struct)
                .publish();
        visionTargetsFieldSpace = NetworkTableInstance.getDefault()
                .getStructArrayTopic("CoralVision/targetPoses",
                        Pose3d.struct)
                .publish();
        scoringPoseFieldSpace = NetworkTableInstance.getDefault().getStructTopic("CoralVision/scoringPose",
                Pose2d.struct).publish();
        cameraPoseFieldSpace = NetworkTableInstance.getDefault().getStructTopic("CoralVision/cameraPoseFieldSpace",
                Pose3d.struct).publish();

        // Simulator for testing/debugging
        if (Robot.isSimulation()) {
            sim = new CoralReefVisionSim();
        }
    }

    @Override
    public void periodic() {
        double[] angles = inputAngles.get();
        double[] distances = inputDistances.get();

        visionTargets.clear();

        if (angles.length == distances.length) {
            visionTargets.clear();
            // Got good data from coprocessor
            for (int i = 0; i < angles.length; i++) {
                double r = distances[i];
                double theta = angles[i] + Constants.Coral.Vision.CAM_YAW.getRadians(); // Radians!!!

                // Calculate vision target positions
                Translation3d targetPos = Constants.Coral.Vision.CAM_POSE.getTranslation().plus(
                        new Translation3d(r * Math.cos(theta), r * Math.sin(theta), 0.0));
                visionTargets.add(targetPos);
            }
        }

        // Select a target
        double minDistance = Double.MAX_VALUE;
        if (visionTargets.size() == 0) {
            selectedTargetIndex = -1;
        } else {
            for (int i = 0; i < visionTargets.size(); ++i) {
                double dist = visionTargets.get(i).toTranslation2d()
                        .getDistance(Constants.Coral.Vision.SCORING_BUMPER_POINT);
                if (dist < minDistance) {
                    minDistance = dist;
                    selectedTargetIndex = i;
                }
            }
        }

        // Filter out too far away targets (1 meter)
        if (minDistance >= Reef.faceOffset * 2.5 + Units.inchesToMeters(10.0)) {
            selectedTargetIndex = -1;
        }

        cameraPublisher.set(Constants.Coral.Vision.CAM_POSE);
        visionTargetPublisher.set(visionTargets.toArray(new Translation3d[] {}));
    }

    @Override
    public void simulationPeriodic() {

    }

    public Translation2d getSelectedTargetError() {
        if (selectedTargetIndex == -1 || selectedTargetIndex >= visionTargets.size()) {
            return new Translation2d();
        } else {
            return visionTargets.get(selectedTargetIndex).toTranslation2d()
                    .minus(Constants.Coral.Vision.SCORING_POSITION);
        }
    }

    public Pose2d getSelectedTargetPose(SwerveSubsystem swerveSub) {
        if (selectedTargetIndex == -1 || selectedTargetIndex >= visionTargets.size()) {
            return new Pose2d();
        } else {
            return swerveSub.getOdometryPose()
                    .transformBy(new Transform2d(
                            visionTargets.get(selectedTargetIndex).toTranslation2d(),
                            Rotation2d.kZero));
        }
    }

    public Pose3d getCameraPoseFieldSpace(SwerveSubsystem swerveSub) {
        return new Pose3d(swerveSub.getOdometryPose())
                .transformBy(new Transform3d(Pose3d.kZero, Constants.Coral.Vision.CAM_POSE));
    }

    public void publishDebugData(SwerveSubsystem swerveSubsystem) {
        scoringPoseFieldSpace.set(swerveSubsystem.getOdometryPose().transformBy(new Transform2d(
                Constants.Coral.Vision.SCORING_POSITION,
                Rotation2d.kZero)));
        cameraPoseFieldSpace.set(getCameraPoseFieldSpace(swerveSubsystem));

        // Transform local vision targets to field space
        Pose3d primaryTargetPose3d = new Pose3d(swerveSubsystem.getOdometryPose());
        Pose3d[] targetsField = new Pose3d[visionTargets.size()];
        for (int i = 0; i < visionTargets.size(); i++) {
            targetsField[i] = new Pose3d(swerveSubsystem.getOdometryPose())
                    .transformBy(new Transform3d(visionTargets.get(i), Rotation3d.kZero));
            if (i == selectedTargetIndex)
                primaryTargetPose3d = targetsField[i];
        }
        visionTargetsFieldSpace.set(targetsField);
        primaryVisionTargetFieldSpace.set(primaryTargetPose3d);

    }
}
