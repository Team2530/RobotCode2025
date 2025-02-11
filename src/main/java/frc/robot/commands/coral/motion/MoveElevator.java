package frc.robot.commands.coral.motion;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class MoveElevator extends Command {
    private CoralSubsystem coralSub;
    private Supplier<CoralPresets> presetSupplier;

    public MoveElevator(CoralSubsystem coralSub, Supplier<CoralPresets> presetSupplier) {
        this.coralSub = coralSub;
        this.presetSupplier = presetSupplier;
        addRequirements(coralSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Arm Sequence", "Moving Elevator");
        coralSub.setCoralPresetElevator(presetSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return Constants.Elevator.DBG_DISABLED || coralSub.isElevatorSupposedToBeInPosition();
    }
}
