package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralIntakePresets;

public class ActivateCoralIntakeCommand extends Command {

    private final CoralSubsystem subsystem;
    private final boolean scoring;
   
    public ActivateCoralIntakeCommand(CoralSubsystem subsystem) {
        this.subsystem = subsystem;
        this.scoring = subsystem.isHolding();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // score if holding, intake if not
        if (scoring) {
            subsystem.setCoralIntakePreset(CoralIntakePresets.SCORE);
        } else {
            subsystem.setCoralIntakePreset(CoralIntakePresets.INTAKE);
        }   
    }

    @Override
    public void end(boolean interrupted) {
        if (scoring) {
            subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
        } else {
            // if intaking but failed
            if (!subsystem.isHolding()) {
                subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
            }
            // else continue to hold the piece
        }
    }

}
