package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSub;

    public enum ElevatorPresets {
        STOW(0.0),
        MIDDLE(Constants.Elevator.PhysicalParameters.elevatorHeightMeters / 2),
        TOP(Constants.Elevator.PhysicalParameters.elevatorHeightMeters);

        private ElevatorPresets(double pos_meters) {
            this.position_m = pos_meters;
        }

        private double position_m;
    }

    private ElevatorPresets target = ElevatorPresets.STOW;
    private double offset;

    public ElevatorCommand(ElevatorSubsystem elevatorSub, ElevatorPresets targetPosition, double targetOffset) {
        this.elevatorSub = elevatorSub;
        this.target = targetPosition;
        this.offset = targetOffset;
        addRequirements(elevatorSub);
    }

    @Override
    public void initialize() {
        double tgt = target.position_m + offset;
        elevatorSub.ppc.setGoal(MathUtil.clamp(tgt, 0, Constants.Elevator.PhysicalParameters.elevatorHeightMeters));
        SmartDashboard.putString("Elevator Command", target.toString());
    }

    @Override
    public boolean isFinished() {
        return elevatorSub.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        // NOTE: Don't disable (so it position-holds with feedforward)
        // elevatorSub.disable();
    }
}
