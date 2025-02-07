package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralIntakePresets;

public class ScoreCoralCommand extends Command{

    private final CoralSubsystem subsystem;
   
    public ScoreCoralCommand(CoralSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setCoralIntakePreset(CoralIntakePresets.SCORE);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isHolding();
    }

}
