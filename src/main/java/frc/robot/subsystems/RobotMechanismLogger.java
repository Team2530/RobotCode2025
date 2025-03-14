package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.HashSet;
import java.util.function.Consumer;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralArm;
import frc.robot.subsystems.coral.CoralSubsystem;

public class RobotMechanismLogger extends SubsystemBase {
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final StructPublisher<Pose3d> s1;
    private final StructPublisher<Pose3d> s2;
    private final StructPublisher<Pose3d> arm;
    private final StructPublisher<Pose3d> wrist1;
    private final StructPublisher<Pose3d> wrist2;
    private final StructPublisher<Pose3d> coral;
    private final StructPublisher<Pose3d> algae;
    private final StructPublisher<Pose3d> algaeArm;

    // Zeroed component poses
    private Pose3d s1Pose = new Pose3d(0, 0, 0, new Rotation3d());
    private Pose3d s2Pose = new Pose3d(0, 0, 0, new Rotation3d());
    private Pose3d armPose = new Pose3d(0, 0, 0, new Rotation3d());
    private Pose3d wrist1Pose = new Pose3d(0, 0, 0, new Rotation3d());
    private Pose3d wrist2Pose = new Pose3d(0, 0, 0, new Rotation3d());
    private Pose3d coralPose = new Pose3d();
    private Pose3d algaeArmPose = new Pose3d();
    private Pose3d algaePose = new Pose3d();

    public RobotMechanismLogger(CoralSubsystem coralSubsystem, SwerveSubsystem swerveSubsystem,
            AlgaeSubsystem algaeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.algaeSubsystem = algaeSubsystem;
        s1 = NetworkTableInstance.getDefault().getStructTopic("0_ElevatorS1", Pose3d.struct).publish();
        s2 = NetworkTableInstance.getDefault().getStructTopic("1_ElevatorS2", Pose3d.struct).publish();
        arm = NetworkTableInstance.getDefault().getStructTopic("2_Arm", Pose3d.struct).publish();
        wrist1 = NetworkTableInstance.getDefault().getStructTopic("3_Wrist1", Pose3d.struct).publish();
        wrist2 = NetworkTableInstance.getDefault().getStructTopic("4_Wrist2", Pose3d.struct).publish();
        coral = NetworkTableInstance.getDefault().getStructTopic("Coral_Pose", Pose3d.struct).publish();
        algae = NetworkTableInstance.getDefault().getStructTopic("Algae_Pose", Pose3d.struct).publish();
        algaeArm = NetworkTableInstance.getDefault().getStructTopic("5_AlgaeArm", Pose3d.struct).publish();
    }

    @Override
    public void periodic() {
        updatePoses();
        updateAdvantageScope();
    }

    public void updateAdvantageScope() {
        s1.set(s1Pose);
        s2.set(s2Pose);
        arm.set(armPose);
        wrist1.set(wrist1Pose);
        wrist2.set(wrist2Pose);
        coral.set(coralPose);
        algae.set(algaePose);
        algaeArm.set(algaeArmPose);
    }

    public void updatePoses() {
        double elevatorHeight = coralSubsystem.getElevator().getPosition();
        double armRotation = coralSubsystem.getPivotPositionDegrees();

        s1Pose = new Pose3d(0.026, 0, 0.055 + elevatorHeight / 2, new Rotation3d());
        s2Pose = new Pose3d(0, 0, 0.081 + elevatorHeight, new Rotation3d());
        armPose = new Pose3d(0.071, 0, 0.22 + elevatorHeight,
                new Rotation3d(Units.degreesToRadians(armRotation), 0, 0));

        wrist1Pose = armPose.transformBy(new Transform3d(0.11, 0, 0.74 - 0.22,
                new Rotation3d(0, 0, -Units.degreesToRadians(
                        coralSubsystem.getCoralArm().getRollPositionDegrees()))));

        wrist2Pose = wrist1Pose.transformBy(new Transform3d(0.019050,
                0.0254,
                0.075057,
                new Rotation3d(0,
                        Units.degreesToRadians(
                                coralSubsystem.getCoralArm().getPitchPositionDegrees()),
                        0)));

        algaeArmPose = armPose.transformBy(new Transform3d(
                0.145, 0.0, 0.365,
                new Rotation3d(-Units
                        .degreesToRadians(algaeSubsystem.getArm().getPositionDegrees() + 10.0),
                        0.0,
                        0.0)));

        if (coralSubsystem.isHolding()) {
            // coralPose = new Pose3d(Units.inchesToMeters(2.0), 0, 0, Rotation3d.kZero)
            // .relativeTo(wrist2Pose.relativeTo(new
            // Pose3d(swerveSubsystem.getOdometryPose())));
            Pose3d coralHoldingPose = new Pose3d(0.0, 0, Units.inchesToMeters(2.5 + 11.875
                    / 2.0),
                    new Rotation3d(0.0, Units.degreesToRadians(90.0), 0.0));
            coralPose = new Pose3d(swerveSubsystem.getOdometryPose())
                    .transformBy(new Transform3d(wrist2Pose.getTranslation(),
                            wrist2Pose.getRotation()))
                    .transformBy(new Transform3d(coralHoldingPose.getTranslation(),
                            coralHoldingPose.getRotation()));
        } else {
            coralPose = Pose3d.kZero;
        }

        if (algaeSubsystem.isHolding()) {
            algaePose = new Pose3d(swerveSubsystem.getOdometryPose())
                    .transformBy(new Transform3d(armPose.getTranslation(), armPose
                            .getRotation()))
                    .transformBy(new Transform3d(0.13, -Units.inchesToMeters(8.125), 0.55,
                            Rotation3d.kZero));
        } else {
            algaePose = Pose3d.kZero;
        }
    }
}
