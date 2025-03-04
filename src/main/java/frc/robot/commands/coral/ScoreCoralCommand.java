package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralIntakePresets;

public class ScoreCoralCommand extends Command {

    private final CoralSubsystem subsystem;
    private double coralExitTime = 0;

    public ScoreCoralCommand(CoralSubsystem subsystem) {
        this.subsystem = subsystem;
        // addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setCoralIntakePreset(CoralIntakePresets.SCORE);
        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("[SIM] Holding Coral", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setCoralIntakePreset(CoralIntakePresets.STOP);
    }

    @Override
    public boolean isFinished() {
        // if (!subsystem.isHolding() && coralExitTime == 0) {
        // coralExitTime = Timer.getFPGATimestamp();
        // }
        // return !subsystem.isHolding()
        // && ((Timer.getFPGATimestamp() - coralExitTime) >
        // Constants.Coral.Intake.SCORE_EXTRA_SECONDS);

        // Just keep going at it!
        // For autos, might want a different command?
        // return Robot.isSimulation();
        return false;
    }

}
