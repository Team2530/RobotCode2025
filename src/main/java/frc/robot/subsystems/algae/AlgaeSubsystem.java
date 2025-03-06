package frc.robot.subsystems.algae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class AlgaeSubsystem extends SubsystemBase {

    public enum AlgaePresets {
        STOW(0.0),
        REMOVE(100.0),
        INTAKE(80.0),
        HOLD(80.0),
        OUT_OF_THE_WAY(30.0);

        public double armAngle;

        private AlgaePresets(double armAngle) {
            this.armAngle = armAngle;
        }
    }

    public enum AlgaeIntakePresets {
        INTAKING(1.0, 20),
        REMOVAL(-1.0, 20),
        PURGE(-1, 20),
        SHOOT(-1, 20),
        HOLD(0.25, 15),
        STOP(0, 20);

        public double outputPercentage;
        public int currentLimitA;

        AlgaeIntakePresets(double outputPercentage, int current) {
            this.outputPercentage = outputPercentage;
            this.currentLimitA = current;
        }
    }

    private final AlgaeArm arm = new AlgaeArm();
    private final AlgaeIntake intake = new AlgaeIntake();

    public AlgaeArm getArm() {
        return arm;
    }

    public AlgaeIntake getIntake() {
        return intake;
    }

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
            intake.setIntakePreset(preset);
            currentIntakePreset = preset;
        }
    }

    public boolean isHolding() {
        return intake.isHolding();
    }
}
