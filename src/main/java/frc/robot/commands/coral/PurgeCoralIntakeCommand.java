package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralIntakePresets;

public class PurgeCoralIntakeCommand extends Command {
    private final CoralSubsystem subsystem;

    public PurgeCoralIntakeCommand(CoralSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        subsystem.setCoralIntakePreset(CoralIntakePresets.PURGE);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
    }
}
