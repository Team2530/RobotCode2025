package frc.robot.subsystems.coral;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Predicate;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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
    private BooleanPublisher hasTargetPublisher;

    // Inputs from vision coprocessor
    NetworkTable visionRawTable = NetworkTableInstance.getDefault().getTable("CoralVision/raw");
    private final DoubleArraySubscriber inputAngles;
    private final DoubleArraySubscriber inputDistances;
    private final IntegerSubscriber inputFrame;

    // private DatagramSocket visionDataRecever;
    // private Alert visionAlert = new Alert("Vision Error", AlertType.kError);
    // private byte[] dataBuf = new byte[1024];

    @Logged
    private ArrayList<Translation3d> visionTargets = new ArrayList<Translation3d>();
    private int selectedTargetIndex = -1;
    private long lastFrame = 0;

    private CoralReefVisionSim sim;

    public CoralReefVision() {
        visionRawTable = NetworkTableInstance.getDefault().getTable("CoralVision/raw");
        inputAngles = visionRawTable.getDoubleArrayTopic("angles").subscribe(new double[] {});
        inputDistances = visionRawTable.getDoubleArrayTopic("distances").subscribe(new double[] {});
        inputFrame = visionRawTable.getIntegerTopic("frame").subscribe(0);

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

        hasTargetPublisher = NetworkTableInstance.getDefault().getBooleanTopic("CoralVision/hasTarget").publish();

        // Simulator for testing/debugging
        if (Robot.isSimulation()) {
            sim = new CoralReefVisionSim();
        }
    }

    @Override
    public void periodic() {

        long frame = inputFrame.getAsLong();
        double[] angles = inputAngles.get();
        double[] distances = inputDistances.get();

        boolean dataUpdated = true;// frame != lastFrame
        if (dataUpdated) {
            lastFrame = frame;

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

            // Filter out invalid vision targets
            visionTargets.removeIf(new Predicate<Translation3d>() {
                @Override
                public boolean test(Translation3d t) {
                    // TODO Auto-generated method stub
                    double camDist = t.getDistance(Constants.Coral.Vision.CAM_POSE.getTranslation());
                    if (camDist > 2.0 || camDist < 0.25)
                        return true;
                    return false;
                }
            });

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

        hasTargetPublisher.set(hasValidTarget());
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

    public boolean hasValidTarget() {
        return selectedTargetIndex != -1;
    }
}
