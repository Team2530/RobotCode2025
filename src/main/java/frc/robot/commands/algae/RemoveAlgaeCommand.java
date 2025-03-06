package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeIntakePresets;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;

public class RemoveAlgaeCommand extends Command {
    private final AlgaeSubsystem subsystem;

    public RemoveAlgaeCommand(AlgaeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setAlgaePreset(AlgaePresets.REMOVE);
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.REMOVAL);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setAlgaePreset(AlgaePresets.STOW);
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.STOP);
    }
}
