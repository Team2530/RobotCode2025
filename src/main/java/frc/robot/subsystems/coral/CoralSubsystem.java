package frc.robot.subsystems.coral;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LimelightAssistance;
import frc.robot.RobotContainer;
import frc.robot.commands.coral.motion.MoveElevator;
import frc.robot.commands.coral.motion.MovePitch;
import frc.robot.commands.coral.motion.MovePivot;
import frc.robot.commands.coral.motion.MoveRoll;
import frc.robot.commands.coral.motion.StowArm;
import frc.robot.commands.coral.motion.WaitArmClearance;
import frc.robot.commands.coral.motion.WaitElevatorApproach;
import frc.robot.commands.coral.motion.WaitRollApproach;
import frc.robot.commands.coral.motion.WaitRollFinished;
import frc.robot.commands.coral.motion.WristAlignAssist;
import frc.robot.commands.coral.motion.WristAlignAssistManual;
import frc.robot.commands.coral.motion.WristStowSafety;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import frc.robot.util.LimelightAssistance;

import frc.robot.util.LimelightContainer;
import frc.robot.util.Reef;

@Logged
public class CoralSubsystem extends SubsystemBase {

    private final CoralArm arm;
    private final CoralIntake intake = new CoralIntake();

    private final CoralElevator elevator = new CoralElevator();

    private final CoralReefVision vision = new CoralReefVision();

    // Members for aim assisting and other automation
    private final XboxController operatorController;
    @NotLogged
    private final SwerveSubsystem swerveSubsystem;

    private final Mechanism2d coralMechanism = new Mechanism2d(2, 3);
    private final MechanismRoot2d rootMechanism = coralMechanism.getRoot("Coral", 1.0, 0.0);
    private final MechanismLigament2d elevatorMechanism = rootMechanism.append(
            new MechanismLigament2d("Elevator", Constants.Elevator.PhysicalParameters.BOTTOM_TO_FLOOR, 90));
    private final MechanismLigament2d pivotMechanism = elevatorMechanism.append(
            new MechanismLigament2d("Coral", Constants.Coral.Pivot.PhysicalConstants.JOINT_LENGTH_METERS, 0));
    private final MechanismLigament2d pitchMechanism = pivotMechanism.append(
            new MechanismLigament2d("Pitch", Constants.Coral.Pitch.PhysicalConstants.JOINT_LENGTH_METERS, 0));
    private final MechanismLigament2d rollMechanism = pivotMechanism.append(
            new MechanismLigament2d("Roll", Constants.Coral.Roll.PhysicalConstants.ARM_LENGTH_METERS, 90));

    StructPublisher<Pose2d> leftPosePub = NetworkTableInstance.getDefault().getStructTopic("Debug/Left", Pose2d.struct)
            .publish();
    StructPublisher<Pose2d> rightPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("Debug/Right", Pose2d.struct)
            .publish();

    public enum CoralPresets {
        LEVEL_1(0.05, Units.radiansToDegrees(0.662), 65, Units.radiansToDegrees(1.41), true),
        LEVEL_2(0.247 - 0.085 - 0.003, 15 - 0.7, 90, 98.0 + 0.7, true, true),
        LEVEL_3(0.650 - 0.085 - 0.003, 15 - 0.7, 90, 98.0
                + 0.7, true,
                true),
        LEVEL_4(1.342 - 0.02, 19.5, 90, 114.5, true,
                true),
        INTAKE(0.097, 18.5, 90, 50, true),
        STOW(0.05, 0.0, 0.0, 0.0, true),
        ZERO(0.0, 0.0, 0.0, 0.0, false),

        ALGAE_REM_LOW(0.62, 32.0, 0.0, 0.0, false),
        ALGAE_REM_HIGH(1.05, 32.0, 0.0, 0.0, false),

        ALGAE_STOW_GROUND(0.05,
                32.0, 90.0, 42.0,
                false),
        ALGAE_STOW_LOW(0.44,
                32.0, 90.0, 42.0,
                false),
        ALGAE_STOW_HIGH(0.822,
                32.0, 90.0, 42.0,
                false),

        // ALGAE_STOW_BARGE(0.9, -20.0, 90.0, 42.0,
        // false),

        ALGAE_ACQUIRE_LOW(0.452, 33.5, 90.0, 42.0, false),
        ALGAE_ACQUIRE_LOLLIPOP(0.05, 39.0, 90.0, 42.0, false),
        ALGAE_ACQUIRE_FLOOR(0.03, 65.0, 90.0, 25.0, false),
        ALGAE_ACQUIRE_HIGH(0.832,
                33.5, 90.0, 42.0, false),

        ALGAE_PROCESSOR(0.03, 38.0, 90.0, -20.0, false),
        ALGAE_BARGE(1.45, -10.0, 90.0, 42.0, false),
        // ALGAE_BARGE(
        // 1.44, -45.0, 90.0, -10.0, false),

        ALGAE_STOW_BUMPER(0.03, 43.0, 90.0, 50.0, false),

        CUSTOM(Double.NaN, Double.NaN, Double.NaN, Double.NaN, false);

        double elevatorHeightM; // Elevator height (relative to bottom of elevator/fully retracted)
        double pivotAngleDeg; // Looking at the robot from the FRONT (algae intake side), positive to the
                              // right, and negative to the left (positive=CW)
        double rollAngleDeg; // Wrist 1 angle, degrees from pointing at the bumpers on the CORAL ARM side of
                             // the robot. positive=CCW
        double pitchAngleDeg; // Wrist 2 angle, degrees from pointing straight up (max: 115deg)
        boolean allowMirror;
        boolean allowAimAssist;

        private CoralPresets(double elevatorHeight, double pivotAngle, double rollAngle, double pitchAngle,
                boolean allowMirror) {
            this.elevatorHeightM = elevatorHeight;
            this.pivotAngleDeg = pivotAngle;
            this.rollAngleDeg = rollAngle;
            this.pitchAngleDeg = pitchAngle;
            this.allowMirror = allowMirror;
            this.allowAimAssist = false;
        }

        private CoralPresets(double elevatorHeight, double pivotAngle, double rollAngle, double pitchAngle,
                boolean allowMirror, boolean allowAimAssist) {
            this(elevatorHeight, pivotAngle, rollAngle, pitchAngle, allowMirror);
            this.allowAimAssist = allowAimAssist;
        }
    }

    @NotLogged
    AlgaeSubsystem algaeSub;

    public CoralSubsystem(SwerveSubsystem swerveSubsystem, XboxController operatorController, AlgaeSubsystem algae) {
        this.swerveSubsystem = swerveSubsystem;
        this.operatorController = operatorController;
        this.algaeSub = algae;
        arm = new CoralArm(this.algaeSub);
    }

    public enum MirrorPresets {
        RIGHT(false),
        LEFT(true);

        boolean isMirrored;

        private MirrorPresets(boolean isMirrored) {
            this.isMirrored = isMirrored;
        }
    }

    public enum CoralIntakePresets {
        INTAKE(1, 40.0),
        HOLD(0.4, 12.5),
        PURGE(-1, 40.0),
        SCORE(-1, 30.0),
        SCORE_L1(-0.1, 30.0),
        STOP(0, 12.5),

        CUSTOM(Double.NaN, 40.0);

        double intakePercentage;
        double intakeCurrent;

        private CoralIntakePresets(double intakePercentage, double intakeCurrent) {
            this.intakePercentage = intakePercentage;
            this.intakeCurrent = intakeCurrent;
        }
    }

    @Override
    public void periodic() {
        // i have no idea what any of the getPositions output
        elevatorMechanism
                .setLength(elevator.getPosition() + Constants.Elevator.PhysicalParameters.CORAL_PIVOT_VERTICAL_OFFSET);
        pivotMechanism.setAngle(arm.getPivotPositionDegrees());
        pitchMechanism.setAngle(arm.getPitchPositionDegrees());
        rollMechanism.setLength(Math.cos(Units.degreesToRadians(arm.getRollPositionDegrees()))
                * Constants.Coral.Roll.PhysicalConstants.JOINT_LENGTH_METERS);

        // SmartDashboard.putData("Coral Mechanism", coralMechanism);
        // SmartDashboard.putBoolean("Elevator in position", isElevatorInPosition());
        // SmartDashboard.putBoolean("Roll in position", isRollInPosition());
        // SmartDashboard.putBoolean("Pitch in position", isPitchInPosition());
        // SmartDashboard.putBoolean("Pivot in position", isPivotInPosition());

        // SmartDashboard.putBoolean("Elevator SUPPOSED to be in position",
        // isElevatorSupposedToBeInPosition());
        // SmartDashboard.putBoolean("Roll SUPPOSED to be in position",
        // isRollSupposedToBeInPosition());
        // SmartDashboard.putBoolean("Pitch SUPPOSED to be in position",
        // isPitchSupposedToBeInPosition());
        // SmartDashboard.putBoolean("Pivot SUPPOSED to be in position",
        // isPivotSupposedToBeInPosition());

        vision.publishDebugData(swerveSubsystem);

        Pose2d left = swerveSubsystem.getOdometryPose().transformBy(new Transform2d(0, 0.2, new Rotation2d()));
        Pose2d right = swerveSubsystem.getOdometryPose().transformBy(new Transform2d(0, -0.2, new Rotation2d()));
        leftPosePub.set(left);
        rightPosePub.set(right);
        // Reef.putToShuffleboard();

    }

    private CoralPresets currentPreset = CoralPresets.STOW;
    private MirrorPresets mirrorSetting = MirrorPresets.RIGHT;
    private CoralIntakePresets currentIntakePreset = CoralIntakePresets.STOP;

    public void setCoralPresetDIRECT(CoralPresets preset) {
        if (preset == CoralPresets.CUSTOM) {
            // uhhh i don't now how to throw an exception and i don't feel like figuring it
            // out
        } else if (preset != currentPreset) {
            elevator.setGoalPosition(preset.elevatorHeightM);
            arm.setPivotGoalDegrees(
                    preset.pivotAngleDeg
                            * (preset.allowMirror ? (mirrorSetting.isMirrored ? -1.0 : 1.0) : 1.0));
            arm.setRollGoalDegrees(preset.rollAngleDeg
                    * (preset.allowMirror ? (mirrorSetting.isMirrored ? -1.0 : 1.0) : 1.0));
            arm.setPitchGoalDegrees(preset.pitchAngleDeg);

            currentPreset = preset;
        }
    }

    public void setCoralPresetElevator(CoralPresets preset) {
        elevator.setGoalPosition(preset.elevatorHeightM);
        currentPreset = preset;
    }

    public CoralPresets getCurrentPreset() {
        return currentPreset;
    }

    public boolean isElevatorInPosition() {
        return elevator.isInPosition();
    }

    public void setCoralPresetPivot(CoralPresets preset) {
        SmartDashboard.putNumber("Pivot Pre", preset.pivotAngleDeg);
        arm.setPivotGoalDegrees(
                preset.pivotAngleDeg
                        * (preset.allowMirror ? (mirrorSetting.isMirrored ? -1 : 1) : 1.0));
        currentPreset = preset;
    }

    public boolean isPivotInPosition() {
        return arm.isPivotInPosition();
    }

    public void setCoralPresetPitch(CoralPresets preset) {
        arm.setPitchGoalDegrees(
                preset.pitchAngleDeg);
        currentPreset = preset;
    }

    public boolean isPitchInPosition() {
        return arm.isPitchInPosition();
    }

    public void setCoralPresetRoll(CoralPresets preset) {
        arm.setRollGoalDegrees(
                preset.rollAngleDeg
                        * (preset.allowMirror ? (mirrorSetting.isMirrored ? -1 : 1) : 1.0));
        currentPreset = preset;
    }

    public boolean isRollInPosition() {
        return arm.isRollInPosition();
    }

    public double getPivotGoalDegrees() {
        return arm.getPivotGoalDegrees();
    }

    public double getRollGoalDegrees() {
        return arm.getRollGoalDegrees();
    }

    public double getPitchGoalDegrees() {
        return arm.getPitchGoalDegrees();
    }

    public boolean isHolding() {
        return Robot.isSimulation() ? SmartDashboard.getBoolean("[SIM] Holding Coral", false) : intake.isHolding();
    }

    public BooleanSupplier isHoldingSupplier() {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isHolding();
            }
        };
    }

    public void setCustomPosition(double elevatorHeight, double pivotAngle, double rollAngle, double pitchAngle) {
        currentPreset = CoralPresets.CUSTOM;

        elevator.setGoalPosition(elevatorHeight);
        arm.setPivotGoalDegrees(pivotAngle);
        arm.setRollGoalDegrees(rollAngle);
        arm.setPitchGoalDegrees(pitchAngle);
    }

    public void setCustomElevatorMeters(double elevatorHeight) {
        currentPreset = CoralPresets.CUSTOM;
        elevator.setGoalPosition(elevatorHeight);
    }

    public void setCustomPivotDegrees(double pivotAngle) {
        currentPreset = CoralPresets.CUSTOM;
        arm.setPivotGoalDegrees(pivotAngle);
    }

    public void setCustomRollDegrees(double rollAngle) {
        currentPreset = CoralPresets.CUSTOM;
        arm.setRollGoalDegrees(rollAngle);
    }

    public void setCustomPitchDegrees(double pitchAngle) {
        currentPreset = CoralPresets.CUSTOM;
        arm.setPitchGoalDegrees(pitchAngle);
    }

    public void mirrorArm() {
        if (mirrorSetting == MirrorPresets.LEFT) {
            mirrorSetting = MirrorPresets.RIGHT;
        } else
            mirrorSetting = MirrorPresets.LEFT;
    }

    public void mirrorArm(MirrorPresets preset) {
        mirrorSetting = preset;
    }

    public MirrorPresets getMirror() {
        return mirrorSetting;
    }

    public void autoSetMirrorIntake() {
        Pose2d robotPose = swerveSubsystem.getOdometryPose();
        Pose2d closestSource = robotPose.nearest(FieldConstants.getSourceSidePoses());
        Pose2d left = robotPose.transformBy(new Transform2d(0, 0.2, new Rotation2d()));
        Pose2d right = robotPose.transformBy(new Transform2d(0, -0.2, new Rotation2d()));
        leftPosePub.set(left);
        rightPosePub.set(right);

        this.mirrorSetting = left.getTranslation().getDistance(closestSource.getTranslation()) < right.getTranslation()
                .getDistance(closestSource.getTranslation()) ? MirrorPresets.LEFT : MirrorPresets.RIGHT;

        SmartDashboard.putString("Mirror Side", mirrorSetting.name());

        // this.mirrorSetting = (this.leftUltrasonic.get() < this.rightUltrasonic.get())
        // ? MirrorPresets.LEFT
        // : MirrorPresets.RIGHT;
    }

    public void autoSetMirrorScoring() {
        Pose2d robotPose = swerveSubsystem.getOdometryPose();
        Pose2d left = robotPose.transformBy(new Transform2d(0, 0.2, new Rotation2d()));
        Pose2d right = robotPose.transformBy(new Transform2d(0, -0.2, new Rotation2d()));
        leftPosePub.set(left);
        rightPosePub.set(right);

        this.mirrorSetting = left.getTranslation().getDistance(FieldConstants.getReefPose().getTranslation()) < right
                .getTranslation()
                .getDistance(FieldConstants.getReefPose().getTranslation()) ? MirrorPresets.LEFT : MirrorPresets.RIGHT;

        SmartDashboard.putString("Mirror Side", mirrorSetting.name());
    }

    public void setCoralIntakePreset(CoralIntakePresets preset) {
        SmartDashboard.putString("Coral Intake Preset", preset.toString());
        if (preset != currentIntakePreset) {
            intake.setOutputPercentage(preset.intakePercentage);
            intake.setStatorLimit(preset.intakeCurrent);
            currentIntakePreset = preset;
        }
    }

    public void setCustomIntakePercent(double percentage) {
        currentIntakePreset = CoralIntakePresets.CUSTOM;
        intake.setOutputPercentage(percentage);
        intake.setStatorLimit(currentIntakePreset.intakeCurrent);
    }

    public double getPivotPositionDegrees() {
        return arm.getPivotPositionDegrees();
    }

    public boolean isElevatorSupposedToBeInPosition() {
        // return elevator.isSupposedToBeInPosition();
        return elevator.isInPosition();
    }

    public boolean isPitchSupposedToBeInPosition() {
        return arm.isPitchSupposedToBeInPosition();
    }

    public boolean isRollSupposedToBeInPosition() {
        return arm.isRollSupposedToBeInPosition();
    }

    public boolean isPivotSupposedToBeInPosition() {
        return arm.isPivotSupposedToBeInPosition();
    }

    public boolean isSupposedToBeInPosition() {
        return isPivotSupposedToBeInPosition() && isRollSupposedToBeInPosition() && isElevatorSupposedToBeInPosition()
                && isPitchSupposedToBeInPosition();
    }

    public CoralArm getCoralArm() {
        return arm;
    }

    public CoralIntake getIntake() {
        return intake;
    }

    public CoralElevator getElevator() {
        return elevator;
    }

    public void simSetHolding(boolean holding) {
        SmartDashboard.putBoolean("[SIM] Holding Coral", holding);
    }

    public Command getGoToLockedPresetCommandV2(AlgaeSubsystem algaeSubsystem,
            Supplier<CoralPresets> currentLockedPresetSupplier) {
        return getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier, true);
    }

    public Command getGoToLockedPresetCommandV2(AlgaeSubsystem algaeSubsystem,
            Supplier<CoralPresets> currentLockedPresetSupplier, boolean autoAlignEnable) {
        return new InstantCommand(() -> {
            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE) {
                this.autoSetMirrorIntake();
                // if (this.mirrorSetting.isMirrored)
                // algaeSubsystem.setAlgaePreset(AlgaePresets.OUT_OF_THE_WAY);
            } else {
                this.autoSetMirrorScoring();
            }

            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        }).andThen(new StowArm(
                this))
                .andThen(new ParallelCommandGroup(
                        new MoveElevator(
                                this, currentLockedPresetSupplier),
                        new MovePivot(
                                this, currentLockedPresetSupplier),
                        new MoveRoll(
                                this, currentLockedPresetSupplier)
                                .andThen(new WristAlignAssist(this, operatorController, swerveSubsystem)
                                        .onlyIf(new BooleanSupplier() {
                                            @Override
                                            public boolean getAsBoolean() {
                                                return currentLockedPresetSupplier.get().allowAimAssist
                                                        && autoAlignEnable;
                                            }
                                        })),
                        new WaitRollApproach(this, 40.0).andThen(
                                new WaitElevatorApproach(
                                        this, 0.4))
                                .andThen(new MovePitch(
                                        this, currentLockedPresetSupplier).andThen(new InstantCommand(() -> {
                                            SmartDashboard.putString("Going to",
                                                    currentLockedPresetSupplier.get().toString() + " - Done");
                                        })))));
    }

    public Command getGoToLockedPresetSideFASTCommand(AlgaeSubsystem algaeSubsystem,
            Supplier<CoralPresets> currentLockedPresetSupplier, MirrorPresets mirrorSide) {
        return new InstantCommand(() -> {

            this.mirrorArm(mirrorSide);
            // if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE &&
            // this.mirrorSetting.isMirrored)
            // algaeSubsystem.setAlgaePreset(AlgaePresets.OUT_OF_THE_WAY);

            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        }).andThen(new StowArm(
                this))
                .andThen(new MoveElevator(
                        this, currentLockedPresetSupplier))
                .andThen(new ParallelCommandGroup(
                        new MovePivot(
                                this, currentLockedPresetSupplier),
                        new MoveRoll(
                                this, currentLockedPresetSupplier),
                        new MovePitch(
                                this, currentLockedPresetSupplier)))
                .andThen(new InstantCommand(() -> {
                    SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString() + " - Done");
                }));
    }

    // Goes to a preset more quickly by moving pitch+pivot+roll at the same time,
    // but can throw coral. Good for intaking
    public Command getGoToLockedPresetFASTCommand(AlgaeSubsystem algaeSubsystem,
            Supplier<CoralPresets> currentLockedPresetSupplier) {
        return new InstantCommand(() -> {

            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE) {

                this.autoSetMirrorIntake();
                // if (!this.mirrorSetting.isMirrored)
                // algaeSubsystem.setAlgaePreset(AlgaePresets.OUT_OF_THE_WAY);
            } else {
                this.autoSetMirrorScoring();
            }
            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        }).andThen(new StowArm(this))
                .andThen(new MoveElevator(
                        this, currentLockedPresetSupplier))
                .andThen(new ParallelCommandGroup(
                        new MovePivot(
                                this, currentLockedPresetSupplier),
                        new MoveRoll(
                                this, currentLockedPresetSupplier),
                        new MovePitch(
                                this, currentLockedPresetSupplier)))
                .andThen(new InstantCommand(() -> {
                    SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString() + " - Done");
                }));
    }

    public Command getGoToLockedPresetAlgaeSafeCommand(AlgaeSubsystem algaeSubsystem,
            Supplier<CoralPresets> currentLockedPresetSupplier) {
        return new InstantCommand(() -> {
            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE) {
                algaeSubsystem.setAlgaePreset(AlgaePresets.OUT_OF_THE_WAY);

                this.autoSetMirrorIntake();
            } else {
                this.autoSetMirrorScoring();
            }
            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        })
                .andThen(new MoveElevator(
                        this, currentLockedPresetSupplier)
                        .alongWith(new MovePivot(this, currentLockedPresetSupplier)))
                .andThen(new ParallelCommandGroup(
                        new MoveRoll(
                                this, currentLockedPresetSupplier),
                        new MovePitch(
                                this, currentLockedPresetSupplier)))
                .andThen(new InstantCommand(() -> {
                    SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString() + " - Done");
                }));
    }

    public CoralReefVision getVisionSubsystem() {
        return vision;
    }
}
