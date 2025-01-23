package frc.robot.subsystems.AlgaeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase{
    
    private final AlgaeArm arm;
    private final AlgaeIntake intake;

    public enum AlgaePresets {
        STOW(10, 0),
        FOO(80, 1);


        double armAngle;
        double intakePercentage;

        private AlgaePresets(double armAngle, double intakePercentage) {
            this.armAngle = armAngle;
            this.intakePercentage = intakePercentage;
        }
    }

    private AlgaePresets currentPreset = AlgaePresets.STOW;

    public Algae(AlgaeArm arm, AlgaeIntake intake) {
        this.arm = arm;
        this.intake = intake;
    }

    public void setAlgaePreset(AlgaePresets preset) {
        if (preset != currentPreset) {
            arm.setGoalDegrees(preset.armAngle);
            intake.setOutputPercentage(preset.intakePercentage);

            currentPreset = preset;
        }
    }
}
