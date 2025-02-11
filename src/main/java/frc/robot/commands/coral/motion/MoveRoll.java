package frc.robot.commands.coral.motion;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class MoveRoll extends Command {
    private CoralSubsystem coralSub;
    private Supplier<CoralPresets> presetSupplier;

    public MoveRoll(CoralSubsystem coralSub, Supplier<CoralPresets> presetSupplier) {
        this.coralSub = coralSub;
        // addRequirements(coralSub);
        this.presetSupplier = presetSupplier;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Arm Sequence", "Moving Roll");
        coralSub.setCoralPresetRoll(presetSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return coralSub.isRollInPosition();
    }
}
