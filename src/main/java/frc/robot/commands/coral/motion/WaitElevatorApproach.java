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
        System.out.printf("Finishing %f meters before\n", metersBefore);
    }

    @Override
    public boolean isFinished() {
        return coralSub.getElevator().getPosition() > (coralSub.getElevator().getGoalPosition() - metersBefore);
    }
}
