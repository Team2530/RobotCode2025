package frc.robot.commands.coral.motion;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;

public class WaitPivotClearance extends Command {
    private CoralSubsystem coralSub;

    public WaitPivotClearance(CoralSubsystem coralSub) {
        this.coralSub = coralSub;
        // addRequirements(coralSub);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralSub.getPivotPositionDegrees()) > Constants.Coral.Pivot.FRAME_BORDER_ANGLE;
    }
}
