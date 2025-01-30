package frc.robot.subsystems.AlgaeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase{
    
    private final AlgaeArm arm;
    private final AlgaeIntake intake;

    public enum AlgaePresets {
        STOW(10),
        FOO(80);

        double armAngle;

        private AlgaePresets(double armAngle) {
            this.armAngle = armAngle;
        }
    }

    public enum AlgaeIntakePresets {
        INTAKING(1),
        PURGE(-1),
        STOP(0);

        double outputPercentage;

        AlgaeIntakePresets(double outputPercentage) {
            this.outputPercentage = outputPercentage;
        } 
    }

    private AlgaePresets currentPreset = AlgaePresets.STOW;
    private AlgaeIntakePresets currentIntakePreset = AlgaeIntakePresets.STOP;

    public Algae(AlgaeArm arm, AlgaeIntake intake) {
        this.arm = arm;
        this.intake = intake;
    }

    public void setAlgaePreset(AlgaePresets preset) {
        if (preset != currentPreset) {
            arm.setGoalDegrees(preset.armAngle);
            currentPreset = preset;
        }
    }

    public void setAlgaeIntakePreset(AlgaeIntakePresets preset) {
        if (preset != currentIntakePreset) {
            intake.setOutputPercentage(preset.outputPercentage);
            currentIntakePreset = preset;
        }
    }
}
