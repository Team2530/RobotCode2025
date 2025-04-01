package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeIntakePresets;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;

public class ShootAlgaeBargeCommand extends Command {
    private final AlgaeSubsystem subsystem;
    private boolean autoExit = true;;

    public ShootAlgaeBargeCommand(AlgaeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    public ShootAlgaeBargeCommand(AlgaeSubsystem subsystem, boolean autoExit) {
        this.subsystem = subsystem;
        this.autoExit = autoExit;
        // addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.SHOOT_BARGE);
        subsystem.setAlgaePreset(AlgaePresets.BARGE);

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Holding Algae", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setAlgaeIntakePreset(AlgaeIntakePresets.STOP);
        // subsystem.setAlgaePreset(AlgaePresets.STOW);
    }

    @Override
    public boolean isFinished() {
        return (subsystem.getIntake().getSensorDistance() > Constants.Algae.Intake.SHOT_THRESHOLD) && this.autoExit;
    }
}
