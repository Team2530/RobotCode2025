package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    
    private final AlgaeArm arm;
    private final AlgaeIntake intake;

    private AlgaePresets currentPreset = AlgaePresets.STOW;
    private AlgaeIntakePresets currentIntakePreset = AlgaeIntakePresets.STOP;

    /**
     * Creates a new Algae subsystem
     */
    public AlgaeSubsystem(AlgaeArm arm, AlgaeIntake intake) {
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

    public enum AlgaePresets {
        STOW(10),
        FOO(80);

        public double armAngle;

        private AlgaePresets(double armAngle) {
            this.armAngle = armAngle;
        }
    }

    public enum AlgaeIntakePresets {
        INTAKING(1),
        PURGE(-1),
        STOP(0);

        public double outputPercentage;

        AlgaeIntakePresets(double outputPercentage) {
            this.outputPercentage = outputPercentage;
        } 
    }
}
