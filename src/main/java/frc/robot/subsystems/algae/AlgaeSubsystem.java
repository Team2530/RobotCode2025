package frc.robot.subsystems.algae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class AlgaeSubsystem extends SubsystemBase {

    public enum AlgaePresets {
        STOW(10),
        FLOOR(90);

        public double armAngle;

        private AlgaePresets(double armAngle) {
            this.armAngle = armAngle;
        }
    }

    public enum AlgaeIntakePresets {
        INTAKING(1),
        PURGE(-1),
        SHOOT(-1),
        STOP(0);

        public double outputPercentage;

        AlgaeIntakePresets(double outputPercentage) {
            this.outputPercentage = outputPercentage;
        }
    }

    private final AlgaeArm arm = new AlgaeArm();
    private final AlgaeIntake intake = new AlgaeIntake();

    private AlgaePresets currentPreset = AlgaePresets.STOW;
    private AlgaeIntakePresets currentIntakePreset = AlgaeIntakePresets.STOP;

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

    public boolean isHolding() {
        return intake.isHolding();
    }
}
