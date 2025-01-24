package frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {

    private final CoralArm arm;
    private final CoralIntake intake;
    private final CoralElevator elevator;

    public enum CoralPresets {
        LEVEL_1(1.0, 20.0, 0.0, 0.0),
        LEVEL_2(2.0, 20.0, 0.0, 0.0),
        LEVEL_3(3.0, 20.0, 0.0, 0.0),
        LEVEL_4(4.0, 20.0, 0.0, 0.0),
        INTAKE(1.0, 110.0, 0.0, 0.0),
        STOW(1.0, 0.0, 0.0, 0.0);

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

    private CoralPresets currentPreset = CoralPresets.STOW;

    public Coral(CoralArm arm, CoralIntake intake, CoralElevator elevator) {
        this.arm = arm;
        this.intake = intake;
        this.elevator = elevator;
    }

    public void setCoralPreset(CoralPresets preset) {
        if (preset != currentPreset) {
            elevator.setGoalPosition(preset.elevatorHeight);
            arm.setPivotGoalDegrees(preset.pivotAngle);
            arm.setRollGoalDegrees(preset.rollAngle);
            arm.setPitchGoalDegress(preset.pitchAngle);

            currentPreset = preset;
        }
    }
}
