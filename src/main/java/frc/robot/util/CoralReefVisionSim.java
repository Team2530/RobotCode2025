package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralReefVisionSim extends SubsystemBase {

    NetworkTable visionRawTable = NetworkTableInstance.getDefault().getTable("CoralVision/raw");
    private final DoubleArrayPublisher simDistPub;
    private final DoubleArrayPublisher simAnglePub;
    private final IntegerPublisher simFramePub;

    long simFrame = 0;

    // Drivetrain input for simulating reef pole positions
    StructSubscriber<Pose2d> botPoseSubscriber = NetworkTableInstance.getDefault()
            .getStructTopic("Odometry Pose", Pose2d.struct).subscribe(Pose2d.kZero);

    StructArrayPublisher<Translation2d> debugPub = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Camera Debug Pose", Translation2d.struct).publish();

    public CoralReefVisionSim() {
        simDistPub = visionRawTable.getDoubleArrayTopic("distances").publish();
        simAnglePub = visionRawTable.getDoubleArrayTopic("angles").publish();
        simFramePub = visionRawTable.getIntegerTopic("frame").publish();

        // for (int i = 0; i < numSimTargets; ++i) {
        // SmartDashboard.putNumber("CoralVisionSim/angle" + Integer.toString(i), 0.0);
        // SmartDashboard.putNumber("CoralVisionSim/distance" + Integer.toString(i),
        // 0.0);
        // }
    }

    @Override
    public void simulationPeriodic() {
        ArrayList<Translation2d> reefPoles = new ArrayList<>();
        for (Translation2d translation2d : Reef.baseTranslations.values()) {
            reefPoles.add(AllianceFlipUtil.apply(translation2d));
        }

        // Bot pose from odometry (NT)
        Pose3d botPose = new Pose3d(botPoseSubscriber.get());
        // Reef pole translations in the camera flat view plane (XY plane, +X camera
        // front)
        ArrayList<Translation2d> reefPolesCameraSpace = new ArrayList<>();
        // Camera pose in field space
        Pose3d cameraPoseFieldSpace = botPose
                .transformBy(new Transform3d(Pose3d.kZero, Constants.Coral.Vision.CAM_POSE));

        for (Translation2d pole : reefPoles) {
            Pose3d polePoseCameraSpace = new Pose3d(new Translation3d(pole), Rotation3d.kZero)
                    .relativeTo(cameraPoseFieldSpace);
            reefPolesCameraSpace.add(polePoseCameraSpace.getTranslation().toTranslation2d());
        }

        debugPub.set(reefPolesCameraSpace.toArray(new Translation2d[] {}));

        double[] anglesPrelim = new double[reefPolesCameraSpace.size()];
        double[] distancesPrelim = new double[reefPolesCameraSpace.size()];
        for (int i = 0; i < reefPolesCameraSpace.size(); ++i) {
            anglesPrelim[i] = reefPolesCameraSpace.get(i).getAngle().getRadians();
            distancesPrelim[i] = reefPolesCameraSpace.get(i).getNorm();
        }

        // Filter to what would actually be visible by the camera
        ArrayList<Double> anglesFinal = new ArrayList<>();
        ArrayList<Double> distancesFinal = new ArrayList<>();
        for (int i = 0; i < anglesPrelim.length; ++i) {
            if (Math.abs(anglesPrelim[i]) <= Constants.Coral.Vision.CAM_FOV_HORIZ / 2.0
                    && distancesPrelim[i] < 2.0 && distancesPrelim[i] > 0.0) {
                distancesFinal.add(distancesPrelim[i]);
                anglesFinal.add(anglesPrelim[i]);
            }
        }

        double[] distances = new double[distancesFinal.size()];
        double[] angles = new double[anglesFinal.size()];
        for (int i = 0; i < angles.length; ++i) {
            angles[i] = anglesFinal.get(i);
            distances[i] = distancesFinal.get(i);
        }

        simDistPub.set(distances);
        simAnglePub.set(angles);
        simFramePub.set(simFrame++);

        Reef.putToShuffleboard();
    }
}
