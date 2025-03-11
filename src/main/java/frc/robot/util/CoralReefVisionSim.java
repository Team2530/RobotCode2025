package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralReefVisionSim extends SubsystemBase {
    private final int numSimTargets = 2;

    private final DoubleArrayPublisher simDistPub;
    private final DoubleArrayPublisher simAnglePub;

    public CoralReefVisionSim() {
        simDistPub = NetworkTableInstance.getDefault().getDoubleArrayTopic("CoralVision/raw/distances").publish();
        simAnglePub = NetworkTableInstance.getDefault().getDoubleArrayTopic("CoralVision/raw/angles").publish();

        for (int i = 0; i < numSimTargets; ++i) {
            SmartDashboard.putNumber("CoralVisionSim/angle" + Integer.toString(i), 0.0);
            SmartDashboard.putNumber("CoralVisionSim/distance" + Integer.toString(i), 0.0);
        }
    }

    @Override
    public void simulationPeriodic() {
        double[] angles = new double[numSimTargets];
        double[] distances = new double[numSimTargets];
        for (int i = 0; i < numSimTargets; ++i) {
            angles[i] = Units.degreesToRadians(
                    SmartDashboard.getNumber("CoralVisionSim/angle" + Integer.toString(i), 0.0));
            distances[i] = SmartDashboard.getNumber("CoralVisionSim/distance" + Integer.toString(i), 0.0);
        }

        simDistPub.set(distances);
        simAnglePub.set(angles);
    }
}
