package frc.robot.commands.coral.motion;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class MovePivot extends Command {
    private CoralSubsystem coralSub;
    private Supplier<CoralPresets> presetSupplier;

    public MovePivot(CoralSubsystem coralSub, Supplier<CoralPresets> presetSupplier) {
        this.coralSub = coralSub;
        this.presetSupplier = presetSupplier;
        // addRequirements(coralSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Arm Sequence", "Moving Pivot");
        SmartDashboard.putString("Move Pivot", "Start");

        coralSub.setCoralPresetPivot(presetSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Move Pivot", "End");
    }

    @Override
    public boolean isFinished() {
        return coralSub.isPivotSupposedToBeInPosition();
    }
}
