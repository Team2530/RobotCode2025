package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
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
        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Holding Coral", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
    }
}
