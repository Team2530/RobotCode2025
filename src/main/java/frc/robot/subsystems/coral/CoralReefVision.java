package frc.robot.subsystems.coral;

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
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Coral.Vision;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

@Logged(strategy = Strategy.OPT_IN)
public class CoralReefVision extends SubsystemBase {

    // Vision targets in robot space
    private final StructArrayPublisher<Translation3d> visionTargetPublisher;
    // Camera pose in robot space
    private final StructPublisher<Pose3d> cameraPublisher;

    // Inputs from vision coprocessor
    private final DoubleArraySubscriber inputAngles;
    private final DoubleArraySubscriber inputDistances;

    private ArrayList<Translation3d> visionTargets = new ArrayList<Translation3d>();

    public CoralReefVision() {
        inputAngles = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("CoralVision/raw/angles").subscribe(new double[] {});
        inputDistances = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("CoralVision/raw/distances").subscribe(new double[] {});

        visionTargetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("CoralVision/targets", Translation3d.struct).publish();
        visionTargetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("CoralVision/cameraPose", Pose3d.struct).publish();
    }

    @Override
    public void periodic() {
        double[] angles = inputAngles.get();
        double[] distances = inputAngles.get();

        visionTargets.clear();

        if (angles.length == distances.length) {
            visionTargets.clear();
            // Got good data from coprocessor
            for (int i = 0; i < angles.length; i++) {
                double r = distances[i];
                double theta = angles[i] + Constants.Coral.Vision.CAM_YAW; // Radians!!!

                // Calculate vision target positions
                Translation3d targetPos = Constants.Coral.Vision.CAM_POSE.getTranslation().plus(
                    new Translation3d(r*Math.cos(theta), r*Math.sin(theta), 0.0)
                )
                visionTargets.add(targetPos);
            }
        }

        cameraPublisher.set(Constants.Coral.Vision.CAM_POSE);
        visionTargetPublisher.set(visionTargets.toArray(new Translation3d[] { }))
    }

    @Override
    public void simulationPeriodic() {
        
    }
}
