package frc.robot.commands.coral.motion;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class WristStowSafety extends Command {
    private CoralSubsystem coralSub;

    public WristStowSafety(CoralSubsystem coralSub) {
        this.coralSub = coralSub;
    }

    @Override
    public void initialize() {
        coralSub.setCoralPresetRoll(CoralPresets.STOW);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralSub.getCoralArm().getRollPositionDegrees()) < 90.0;
    }
}
