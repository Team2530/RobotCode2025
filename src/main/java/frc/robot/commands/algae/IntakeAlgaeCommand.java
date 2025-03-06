package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeIntakePresets;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;

public class IntakeAlgaeCommand extends Command {
    private final AlgaeSubsystem subsystem;

    public IntakeAlgaeCommand(AlgaeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.INTAKING);
        subsystem.setAlgaePreset(AlgaePresets.INTAKE);
    }

    @Override
    public boolean isFinished() {
        return subsystem.isHolding();
    }

    @Override
    public void end(boolean interrupted) {
        if (subsystem.isHolding()) {
            subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.HOLD);
            subsystem.setAlgaePreset(AlgaePresets.HOLD);
        } else {
            subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.STOP);
            subsystem.setAlgaePreset(AlgaePresets.STOW);
        }
    }
}
