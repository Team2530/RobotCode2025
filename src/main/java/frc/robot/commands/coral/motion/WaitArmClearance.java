package frc.robot.commands.coral.motion;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class WaitArmClearance extends Command {
    private CoralSubsystem coralSub;

    public WaitArmClearance(CoralSubsystem coralSub) {
        this.coralSub = coralSub;
        // addRequirements(coralSub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralSub.getPivotPositionDegrees()) > Units
                .radiansToDegrees(Constants.Coral.Pivot.ELEVATOR_BORDER_ANGLE);
    }
}
