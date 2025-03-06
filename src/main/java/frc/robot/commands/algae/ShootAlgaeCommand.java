package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeIntakePresets;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;

public class ShootAlgaeCommand extends Command {
    private final AlgaeSubsystem subsystem;

    public ShootAlgaeCommand(AlgaeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.SHOOT);
        subsystem.setAlgaePreset(AlgaePresets.HOLD);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.STOP);
        subsystem.setAlgaePreset(AlgaePresets.STOW);
    }

    @Override
    public boolean isFinished() {
        return subsystem.getIntake().getSensorDistance() > Constants.Algae.Intake.SHOT_THRESHOLD;
    }
}
