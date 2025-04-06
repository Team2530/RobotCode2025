package frc.robot.commands.coral.motion;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

public class WaitElevatorApproach extends Command {
    private CoralSubsystem coralSub;
    private double metersBefore;

    public WaitElevatorApproach(CoralSubsystem coralSub, double metersBefore) {
        this.coralSub = coralSub;
        this.metersBefore = metersBefore;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("WaitElevatorApproach", "waiting " + (Double.toString(metersBefore)) + "m");
    }

    @Override
    public boolean isFinished() {
        boolean finished = coralSub.getElevator()
                .getPosition() > (coralSub.getElevator().getGoalPosition() - metersBefore);
        if (finished) {
            SmartDashboard.putString("WaitElevatorApproach", "finished");
        }
        return finished;
    }
}
