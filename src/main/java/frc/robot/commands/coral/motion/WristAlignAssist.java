package frc.robot.commands.coral.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralReefVision;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class WristAlignAssist extends Command {
    CoralSubsystem coralSubsystem;
    CoralPresets startingPreset;
    CoralReefVision vision;

    public WristAlignAssist(CoralSubsystem coralSub) {
        coralSubsystem = coralSub;
        vision = coralSub.getVisionSubsystem();
    }

    @Override
    public void initialize() {
        startingPreset = coralSubsystem.getCurrentPreset();
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            Translation2d error = vision.getSelectedTargetError();
            coralSubsystem.setCustomRollDegrees(MathUtil.clamp(
                    90.0 - Units.radiansToDegrees(Math.atan2(error.getX(), -error.getY() + Units.inchesToMeters(14.0))),
                    90 - 30.0,
                    90 + 30.0));

            SmartDashboard.putNumber("Error X", error.getX());
            SmartDashboard.putNumber("Error Y", error.getY());
            // coralSubsystem.setCustomPitchDegrees(0);
        }
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
