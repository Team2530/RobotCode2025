package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.coral.CoralSubsystem;

public class WristFollowCommand extends Command {
   
    private final CoralSubsystem subsystem;
    private final CommandXboxController operator;
    
    public WristFollowCommand(CoralSubsystem subsystem, CommandXboxController operator) {
        this.subsystem = subsystem;
        this.operator = operator;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setCustomPitchDegrees((operator.getRightY() * 10) + subsystem.getPitchGoalDegrees());
        subsystem.setCustomRollDegrees((operator.getRightX() * 10) + subsystem.getRollGoalDegrees());
    }
}