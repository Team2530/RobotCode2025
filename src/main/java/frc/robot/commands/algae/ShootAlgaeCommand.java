package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeIntakePresets;

public class ShootAlgaeCommand extends Command {
    private final AlgaeSubsystem subsystem;

    public ShootAlgaeCommand(AlgaeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.SHOOT);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.STOP);
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isHolding();
    }
}
