package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralIntakePresets;

public class IntakeCoralCommand extends Command {
    private final CoralSubsystem subsystem;

    public IntakeCoralCommand(CoralSubsystem subsystem) {
        this.subsystem = subsystem;
        // addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setCoralIntakePreset(CoralIntakePresets.INTAKE);
        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Holding Coral", true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // if intaking failed
        if (!subsystem.isHolding()) {
            subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
            SmartDashboard.putString("Intake Command", "Stopped");
        } else {
            SmartDashboard.putString("Intake Command", "Holding");

            subsystem.setCoralIntakePreset(CoralIntakePresets.HOLD);
        }
        // else continue to hold the piece
    }

    @Override
    public boolean isFinished() {
        return subsystem.isHolding();
    }
}
