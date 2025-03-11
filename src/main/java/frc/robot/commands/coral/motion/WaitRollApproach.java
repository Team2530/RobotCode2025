package frc.robot.commands.coral.motion;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class WaitRollApproach extends Command {
    private CoralSubsystem coralSub;
    double degreesBefore;

    public WaitRollApproach(CoralSubsystem coralSub, double degreesBefore) {
        this.coralSub = coralSub;
        this.degreesBefore = degreesBefore;
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return (Math.abs(coralSub.getRollGoalDegrees()) > 10) && (Math
                .abs(coralSub.getRollGoalDegrees() - coralSub.getCoralArm().getRollPositionDegrees()) < degreesBefore);
    }
}
