package frc.robot.subsystems.coral;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
// import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LimelightContainer;

public class CoralSubsystem extends SubsystemBase {

    private final CoralArm arm = new CoralArm();
    private final CoralIntake intake = new CoralIntake();
    private final CoralElevator elevator = new CoralElevator();

    private final Mechanism2d coralMechanism = new Mechanism2d(2, 3);
    private final MechanismRoot2d rootMechanism = coralMechanism.getRoot("Coral", 0.0, 0.0);
    private final MechanismLigament2d elevatorMechanism = rootMechanism.append(
            new MechanismLigament2d("Elevator", Constants.Elevator.PhysicalParameters.BOTTOM_TO_FLOOR, 0));
    private final MechanismLigament2d pivotMechanism = elevatorMechanism.append(
            new MechanismLigament2d("Coral", Constants.Coral.Pivot.PhysicalConstants.JOINT_LENGTH_METERS, 0));

    private final AnalogPotentiometer leftUltrasonic = new AnalogPotentiometer(Constants.Coral.LEFT_ULTRASONIC_PORT,
            254.0, 0.0);
    private final AnalogPotentiometer rightUltrasonic = new AnalogPotentiometer(Constants.Coral.RIGHT_ULTRASONIC_PORT,
            254.0, 0.0);

    public enum CoralPresets {
        LEVEL_1(0.05, Units.radiansToDegrees(0.635), Units.radiansToDegrees(1.0), Units.radiansToDegrees(1.636)),
        LEVEL_2(0.247, 19.032, 90, 98.068),
        LEVEL_3(0.650, 19.032, 90, 98.068),
        LEVEL_4(1.342, 21.238, 90, 110.062),
        INTAKE(0.05, 18.0, 90, 30.0),
        STOW(0.05, 0.0, 0.0, 0.0),

        CUSTOM(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

        double elevatorHeightM; // Elevator height (relative to bottom of elevator/fully retracted)
        double pivotAngleDeg; // Looking at the robot from the FRONT (algae intake side), positive to the
                              // right, and negative to the left (positive=CW)
        double rollAngleDeg; // Wrist 1 angle, degrees from pointing at the bumpers on the CORAL ARM side of
                             // the robot. positive=CCW
        double pitchAngleDeg; // Wrist 2 angle, degrees from pointing straight up (max: 115deg)

        private CoralPresets(double elevatorHeight, double pivotAngle, double rollAngle, double pitchAngle) {
            this.elevatorHeightM = elevatorHeight;
            this.pivotAngleDeg = pivotAngle;
            this.rollAngleDeg = rollAngle;
            this.pitchAngleDeg = pitchAngle;
        }
    }

    public CoralSubsystem() {
        // Epilogue.bind(this);
    }

    public enum MirrorPresets {
        RIGHT(false),
        LEFT(true),
        STARBOARD(false),
        PORT(true);

        boolean isMirrored;

        private MirrorPresets(boolean isMirrored) {
            this.isMirrored = isMirrored;
        }
    }

    public enum CoralIntakePresets {
        INTAKE(1),
        HOLD(0.4),
        PURGE(-1),
        SCORE(-1),
        STOP(0),

        CUSTOM(Double.NaN);

        double intakePercentage;

        private CoralIntakePresets(double intakePercentage) {
            this.intakePercentage = intakePercentage;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ultra Left", this.leftUltrasonic.get());
        SmartDashboard.putNumber("Ultra Right", this.rightUltrasonic.get());

        // i have no idea what any of the getPositions output
        elevatorMechanism
                .setLength(elevator.getPosition() + Constants.Elevator.PhysicalParameters.CORAL_PIVOT_VERTICAL_OFFSET);
        pivotMechanism.setAngle(arm.getPivotPositionDegrees());

        SmartDashboard.putData("Coral Mechanism", coralMechanism);
        SmartDashboard.putBoolean("Elevator in position", isElevatorInPosition());
        SmartDashboard.putBoolean("Roll in position", isRollInPosition());
        SmartDashboard.putBoolean("Pitch in position", isPitchInPosition());
        SmartDashboard.putBoolean("Pivot in position", isPivotInPosition());

        SmartDashboard.putBoolean("Elevator SUPPOSED to be in position", isElevatorSupposedToBeInPosition());
        SmartDashboard.putBoolean("Roll SUPPOSED to be in position", isRollSupposedToBeInPosition());
        SmartDashboard.putBoolean("Pitch SUPPOSED to be in position", isPitchSupposedToBeInPosition());
        SmartDashboard.putBoolean("Pivot SUPPOSED to be in position", isPivotSupposedToBeInPosition());
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
                            * (mirrorSetting.isMirrored ? -1 : 1));
            arm.setRollGoalDegrees(preset.rollAngleDeg);
            arm.setPitchGoalDegrees(preset.pitchAngleDeg);

            currentPreset = preset;
        }
    }

    public void setCoralPresetElevator(CoralPresets preset) {
        elevator.setGoalPosition(preset.elevatorHeightM);
        currentPreset = preset;
    }

    public boolean isElevatorInPosition() {
        return elevator.isInPosition();
    }

    public void setCoralPresetPivot(CoralPresets preset) {
        SmartDashboard.putNumber("Pivot Pre", preset.pivotAngleDeg);
        arm.setPivotGoalDegrees(
                preset.pivotAngleDeg
                        * (mirrorSetting.isMirrored ? -1 : 1));
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
                        * (mirrorSetting.isMirrored ? -1 : 1));
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

    // public void mirrorArm() {
    // if (LimelightContainer.isReefOnLeft()) {
    // mirrorSetting = MirrorPresets.LEFT;
    // } else {
    // mirrorSetting = MirrorPresets.RIGHT;
    // }
    // }

    public void mirrorArm(MirrorPresets preset) {
        mirrorSetting = preset;
    }

    public void autoSetMirror() {
        this.mirrorSetting = (this.leftUltrasonic.get() < this.rightUltrasonic.get()) ? MirrorPresets.LEFT
                : MirrorPresets.RIGHT;
    }

    public void setCoralIntakePreset(CoralIntakePresets preset) {
        SmartDashboard.putString("Coral Intake Preset", preset.toString());
        if (preset != currentIntakePreset) {
            intake.setOutputPercentage(preset.intakePercentage);
            currentIntakePreset = preset;
        }
    }

    public void setCustomIntakePercent(double percentage) {
        currentIntakePreset = CoralIntakePresets.CUSTOM;
        intake.setOutputPercentage(percentage);
    }

    public double getPivotPositionDegrees() {
        return arm.getPivotPositionDegrees();
    }

    public boolean isElevatorSupposedToBeInPosition() {
        return elevator.isSupposedToBeInPosition();
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
}
