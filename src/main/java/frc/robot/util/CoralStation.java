package frc.robot.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class CoralStation {
    public static final Pose2d[] coralStations = new Pose2d[] {
            new Pose2d(
                    0.85,
                    0.65,
                    Rotation2d.fromDegrees(144.011392 - 90.0)),
            new Pose2d(
                    0.85,
                    8.020 - 0.65,
                    Rotation2d.fromDegrees(-144.011392 + 90.0)),
    };

    public static void putToShuffleboard() {
        System.out.println();
        NetworkTableInstance.getDefault()
                .getStructTopic("Coral Station Right", Pose2d.struct).publish().set(coralStations[0]);
        NetworkTableInstance.getDefault()
                .getStructTopic("Coral Station Left", Pose2d.struct).publish().set(coralStations[1]);

    }
}
