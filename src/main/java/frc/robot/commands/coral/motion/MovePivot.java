package frc.robot.commands.coral.motion;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class MovePivot extends Command {
    private CoralSubsystem coralSub;

    public MovePivot(CoralSubsystem coralSub, Supplier<CoralPresets> presetSupplier) {
        this.coralSub = coralSub;
        addRequirements(coralSub);
        SmartDashboard.putString("Arm Sequence", "Moving Pivot");
        coralSub.setCoralPresetPivot(presetSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return coralSub.isPivotInPosition();
    }
}
