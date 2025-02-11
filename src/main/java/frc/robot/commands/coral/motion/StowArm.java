package frc.robot.commands.coral.motion;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class StowArm extends Command {
    private CoralSubsystem coralSub;

    public StowArm(CoralSubsystem coralSub) {
        this.coralSub = coralSub;
        addRequirements(coralSub);
        SmartDashboard.putString("Arm Sequence", "Stowing Arm");
        coralSub.setCoralPresetPitch(CoralPresets.STOW);
        coralSub.setCoralPresetRoll(CoralPresets.STOW);
        coralSub.setCoralPresetPivot(CoralPresets.STOW);
    }

    @Override
    public boolean isFinished() {
        // Intentionally not worrying about pitch because it doesn't really matter
        // anyways
        return true;
    }
}
