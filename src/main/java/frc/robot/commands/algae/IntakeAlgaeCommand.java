package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeIntakePresets;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;

public class IntakeAlgaeCommand extends Command {
    private final AlgaeSubsystem subsystem;
    private boolean isGround = false;

    public IntakeAlgaeCommand(AlgaeSubsystem subsystem) {
        this(subsystem, false);
    }

    public IntakeAlgaeCommand(AlgaeSubsystem subsystem, boolean floor) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
        this.isGround = floor;
    }

    @Override
    public void initialize() {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.INTAKING);
        subsystem.setAlgaePreset(isGround ? AlgaePresets.INTAKE_FLOOR : AlgaePresets.INTAKE);

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Holding Algae", true);
        }
    }

    @Override
    public boolean isFinished() {
        return subsystem.isHolding();
    }

    @Override
    public void end(boolean interrupted) {
        if (subsystem.isHolding() || DriverStation.isAutonomous()) {
            subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.HOLD);
            subsystem.setAlgaePreset(AlgaePresets.HOLD);
        } else {
            subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.STOP);
            subsystem.setAlgaePreset(AlgaePresets.STOW);
        }
    }
}
