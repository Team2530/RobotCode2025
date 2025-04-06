package frc.robot.commands.coral.motion;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class WaitRollFinished extends Command {
    private CoralSubsystem coralSub;

    public WaitRollFinished(CoralSubsystem coralSub) {
        this.coralSub = coralSub;
        // addRequirements(coralSub);
        SmartDashboard.putString("WaitRollFinish", "waiting");
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        boolean finished = (Math.abs(coralSub.getRollGoalDegrees()) > 10) && coralSub.isRollSupposedToBeInPosition();
        if (finished) {
            SmartDashboard.putString("WaitRollFinish", "finished");
        }
        return finished;
    }
}
