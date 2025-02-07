package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralIntakePresets;

public class IntakeCoralCommand extends Command{
    private final CoralSubsystem subsystem;

    public IntakeCoralCommand(CoralSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setCoralIntakePreset(CoralIntakePresets.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        // if intaking failed
        if (!subsystem.isHolding()) {
            subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
        }
        // else continue to hold the piece
    }

    @Override
    public boolean isFinished() {
        return subsystem.isHolding();
    }
}
