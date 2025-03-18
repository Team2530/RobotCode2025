package frc.robot.commands.coral.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import frc.robot.subsystems.coral.CoralSubsystem.MirrorPresets;

public class WristAlignAssistManual extends Command {
    CoralSubsystem coralSubsystem;
    CoralPresets startingPreset;
    XboxController operatorController;

    public WristAlignAssistManual(CoralSubsystem coralSub, XboxController operatorController) {
        coralSubsystem = coralSub;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        startingPreset = coralSubsystem.getCurrentPreset();
    }

    @Override
    public void execute() {

        coralSubsystem
                .setCustomRollDegrees((coralSubsystem.getMirror() == MirrorPresets.LEFT ? -1.0 : 1.0) * MathUtil.clamp(
                        (90.0 - Units
                                .radiansToDegrees(
                                        Math.atan2((coralSubsystem.getMirror() == MirrorPresets.LEFT ? 1.0 : -1.0)
                                                * operatorController.getRightX(), 2.0))),
                        90 - 30.0,
                        90 + 30.0));
    }

    @Override
    public boolean isFinished() {
        return !(startingPreset == CoralPresets.LEVEL_4 || startingPreset == CoralPresets.LEVEL_2
                || startingPreset == CoralPresets.LEVEL_3);
    }

    @Override
    public void end(boolean interrupted) {
        // Restore roll setting
        coralSubsystem.setCoralPresetRoll(startingPreset);
    }
}
