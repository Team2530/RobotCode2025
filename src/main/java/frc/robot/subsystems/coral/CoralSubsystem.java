package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    private final CoralArm arm;
    private final CoralIntake intake;
    private final CoralElevator elevator;

    private final Mechanism2d coralMechanism = new Mechanism2d(2,3);
    private final MechanismRoot2d rootMechanism = coralMechanism.getRoot("Coral", 1.5, 0);
    private final MechanismLigament2d elevatorMechanism = rootMechanism.append(
        new MechanismLigament2d("Elevator", 1, 0)
    );
    private final MechanismLigament2d pivotMechanism = elevatorMechanism.append(
        new MechanismLigament2d("Coral", 1, 0)
    );

    public enum CoralPresets {
        LEVEL_1(1.0, 20.0, 0.0, 0.0),
        LEVEL_2(2.0, 20.0, 0.0, 0.0),
        LEVEL_3(3.0, 20.0, 0.0, 0.0),
        LEVEL_4(4.0, 20.0, 0.0, 0.0),
        INTAKE(1.0, 110.0, 0.0, 0.0),
        STOW(1.0, 0.0, 0.0, 0.0),

        CUSTOM(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

        double elevatorHeight;
        double pivotAngle;
        double rollAngle;
        double pitchAngle;

        private CoralPresets(double elevatorHeight, double pivotAngle, double rollAngle, double pitchAngle) {
            this.elevatorHeight = elevatorHeight;
            this.pivotAngle = pivotAngle;
            this.rollAngle = rollAngle;
            this.pitchAngle = pitchAngle;
        }
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
        INTAKING(1),
        PURGE(-1),
        STOP(0),

        CUSTOM(Double.NaN);

        double intakePercentage;
        private CoralIntakePresets(double intakePercentage) {
            this.intakePercentage = intakePercentage;
        }
    }

    @Override
    public void periodic() {
        // i have no idea what any of the getPositions output
        elevatorMechanism.setLength(elevator.getPosition());
        pivotMechanism.setAngle(arm.getPivotPositionDegrees());

        SmartDashboard.putData("Coral Mechanism", coralMechanism);
    }

    private CoralPresets currentPreset = CoralPresets.STOW;
    private MirrorPresets mirrorSetting = MirrorPresets.RIGHT;
    private CoralIntakePresets currentIntakePreset = CoralIntakePresets.STOP;

    public CoralSubsystem(CoralArm arm, CoralIntake intake, CoralElevator elevator) {
        this.arm = arm;
        this.intake = intake;
        this.elevator = elevator;
    }

    public void setCoralPreset(CoralPresets preset) {
        if (preset == CoralPresets.CUSTOM) {
            // uhhh i don't now how to throw an exception and i don't feel like figuring it out
        } else if (preset != currentPreset) {
            elevator.setGoalPosition(preset.elevatorHeight);
            arm.setPivotGoalDegrees(
                preset.pivotAngle 
                * (mirrorSetting.isMirrored ? -1 : 1)
            );
            arm.setRollGoalDegrees(preset.rollAngle);
            arm.setPitchGoalDegrees(preset.pitchAngle);

            currentPreset = preset;
        }
    }

    public void setCustomPosition(double elevatorHeight, double pivotAngle, double rollAngle, double pitchAngle) {
        currentPreset = CoralPresets.CUSTOM;

        elevator.setGoalPosition(elevatorHeight);
        arm.setPivotGoalDegrees(pivotAngle);
        arm.setRollGoalDegrees(rollAngle);
        arm.setPitchGoalDegrees(pitchAngle);
    }

    public void setCustomElevatorPosition(double elevatorHeight) {
        currentPreset = CoralPresets.CUSTOM;
        elevator.setGoalPosition(elevatorHeight);
    }

    public void setCustomPivotPosition(double pivotAngle) {
        currentPreset = CoralPresets.CUSTOM;
        arm.setPivotGoalDegrees(pivotAngle);
    }

    public void setCustomRollPosition(double rollAngle) {
        currentPreset = CoralPresets.CUSTOM;
        arm.setRollGoalDegrees(rollAngle);
    }

    public void setCustomPitchPosition(double pitchAngle) {
        currentPreset = CoralPresets.CUSTOM;
        arm.setPitchGoalDegrees(pitchAngle);
    }

    public void mirrorArm() {
        if (mirrorSetting.isMirrored) {
            mirrorSetting = MirrorPresets.RIGHT;
        } else {
            mirrorSetting = MirrorPresets.LEFT;
        }
    }

    public void mirrorArm(MirrorPresets preset) {
        mirrorSetting = preset;
    }

    public void setCoralIntakePreset(CoralIntakePresets preset) {
        if (preset != currentIntakePreset) {
            intake.setOutputPercentage(preset.intakePercentage);
            currentIntakePreset = preset;
        }
    }

    public void setCustomIntakePercent(double percentage) {
        currentIntakePreset = CoralIntakePresets.CUSTOM;
        intake.setOutputPercentage(percentage);
    }
}
 