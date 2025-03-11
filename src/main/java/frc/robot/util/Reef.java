package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;

public class Reef {
    public enum ReefBranch {
        A, B, C, D, E, F, G, H, I, J, K, L;
    }

    private static final Pose2d[] centerFaces = new Pose2d[] {
            new Pose2d(
                    Units.inchesToMeters(144.003),
                    Units.inchesToMeters(158.500),
                    Rotation2d.fromDegrees(180)),
            new Pose2d(
                    Units.inchesToMeters(160.373),
                    Units.inchesToMeters(186.857),
                    Rotation2d.fromDegrees(120)),
            new Pose2d(
                    Units.inchesToMeters(193.116),
                    Units.inchesToMeters(186.858),
                    Rotation2d.fromDegrees(60)),
            new Pose2d(
                    Units.inchesToMeters(209.489),
                    Units.inchesToMeters(158.502),
                    Rotation2d.fromDegrees(0)),
            new Pose2d(
                    Units.inchesToMeters(193.118),
                    Units.inchesToMeters(130.145),
                    Rotation2d.fromDegrees(300)),
            new Pose2d(
                    Units.inchesToMeters(160.375),
                    Units.inchesToMeters(130.144),
                    Rotation2d.fromDegrees(240))
    };

    public static final Translation2d center = new Translation2d(
            4.485,
            4.025);
    // Starting off facing DS wall
    public static final double centerOffset = Units.inchesToMeters(32);
    public static final double faceOffset = Units.inchesToMeters(6.469);
    public static final double poleInset = Units.inchesToMeters(2.0);
    public static final double baseInset = Units.inchesToMeters(12.0);

    public static final Map<ReefBranch, Pose2d> robotBranchPoses = new HashMap<ReefBranch, Pose2d>() {
        {
            ReefBranch[] branchName = ReefBranch.values();
            for (int i = 0; i < 12; i += 2) {
                Pose2d face = centerFaces[i / 2];
                put(branchName[i], face.transformBy(
                        new Transform2d(DriveConstants.FULL_ROBOT_WIDTH / 2.0, faceOffset,
                                new Rotation2d())));
                put(branchName[i + 1], face.transformBy(
                        new Transform2d(DriveConstants.FULL_ROBOT_WIDTH / 2.0, -faceOffset,
                                new Rotation2d())));
            }
        }
    };

    public static final Map<ReefBranch, Translation2d> branchTranslations = new HashMap<ReefBranch, Translation2d>() {
        {
            ReefBranch[] branchName = ReefBranch.values();
            for (int i = 0; i < 12; i += 2) {
                Pose2d face = centerFaces[i / 2];
                put(branchName[i], face.transformBy(
                        new Transform2d(-poleInset, faceOffset,
                                new Rotation2d()))
                        .getTranslation());
                put(branchName[i + 1], face.transformBy(
                        new Transform2d(-poleInset, -faceOffset,
                                new Rotation2d()))
                        .getTranslation());
            }
        }
    };

    public static final Map<ReefBranch, Translation2d> baseTranslations = new HashMap<ReefBranch, Translation2d>() {
        {
            ReefBranch[] branchName = ReefBranch.values();
            for (int i = 0; i < 12; i += 2) {
                Pose2d face = centerFaces[i / 2];
                put(branchName[i], face.transformBy(
                        new Transform2d(-baseInset, faceOffset,
                                new Rotation2d()))
                        .getTranslation());
                put(branchName[i + 1], face.transformBy(
                        new Transform2d(-baseInset, -faceOffset,
                                new Rotation2d()))
                        .getTranslation());
            }
        }
    };

    public static Pose2d getBranchPose2d(ReefBranch branch) {
        return robotBranchPoses.get(branch);
    }

    /**
     * Used to put a pose on Shuffleboard for debugging - Don't repeatadly call
     * this!
     * 
     * @param name Name of pose
     * @param pose Pose2d pose
     */
    public static void pushPoseToShuffleboard(String name, Pose2d pose) {
        StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
                .getStructTopic(name, Pose2d.struct)
                .publish();
        publisher.set(pose);
    }

    static StructArrayPublisher<Pose2d> botPosePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Reef Robot Poses", Pose2d.struct).publish();
    static StructArrayPublisher<Translation2d> branchPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Reef Branches", Translation2d.struct).publish();
    static StructArrayPublisher<Translation2d> basePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Reef Bases", Translation2d.struct).publish();

    /**
     * Just for visualization for poses
     */
    public static void putToShuffleboard() {
        // for (ReefBranch branch : branches.keySet()) {
        // System.out.println(branch.name());
        // Pose2d branchPosition = branches.get(branch);

        // StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        // .getStructTopic(branch.name(), Pose2d.struct).publish();

        // publisher.set(branchPosition);
        // }

        botPosePublisher.set(robotBranchPoses.values().toArray(new Pose2d[] {}));
        branchPublisher.set(branchTranslations.values().toArray(new Translation2d[] {}));
        basePublisher.set(baseTranslations.values().toArray(new Translation2d[] {}));
    }
}