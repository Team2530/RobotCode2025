package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorFollowCommand extends Command {
    private final ElevatorSubsystem elevatorSub;
    private double lastTarget = 0;
    private double lastTime = 0;
    private DoubleSupplier target;
    private SlewRateLimiter limiter = new SlewRateLimiter(Constants.Elevator.PID.MAX_ACCELERATION / 10.0);

    public ElevatorFollowCommand(ElevatorSubsystem elevatorSub, DoubleSupplier target) {
        this.elevatorSub = elevatorSub;
        this.target = target;
        addRequirements(elevatorSub);
    }

    @Override
    public void initialize() {
        elevatorSub.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        double tgt = target.getAsDouble();
        double dX = tgt - lastTarget;
        double dt = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();
        lastTarget = tgt;
        elevatorSub.setGoal(new TrapezoidProfile.State(
                MathUtil.clamp(limiter.calculate(tgt), 0,
                        Constants.Elevator.PhysicalParameters.elevatorHeightMeters),
                0.0));
    }

    @Override
    public void end(boolean interrupted) {
        // NOTE: Don't disable (so it position-holds with feedforward)
        // elevatorSub.disable();
    }
}
