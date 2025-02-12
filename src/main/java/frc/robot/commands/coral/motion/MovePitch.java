package frc.robot.commands.coral.motion;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class MovePitch extends Command {
    private CoralSubsystem coralSub;
    private Supplier<CoralPresets> presetSupplier;

    public MovePitch(CoralSubsystem coralSub, Supplier<CoralPresets> presetSupplier) {
        this.coralSub = coralSub;
        this.presetSupplier = presetSupplier;
        // HACK: This is much sketch
        // addRequirements(coralSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Arm Sequence", "Moving Pitch");
        SmartDashboard.putString("Move Pitch", "Start");

        coralSub.setCoralPresetPitch(presetSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Move Pitch", "End");
    }

    @Override
    public boolean isFinished() {
        return coralSub.isPitchSupposedToBeInPosition();
    }
}
