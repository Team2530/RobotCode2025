package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.Elevator.PID.MAX_ACCELERATION;
import static frc.robot.Constants.Elevator.PhysicalParameters.elevatorHeightMeters;

public class ElevatorFollowCommand extends Command {
    // A reference to the elevator subsystem
    private final ElevatorSubsystem elevatorSub;

    private DoubleSupplier target;

    // See the {@link execute()} method
    private double lastTarget = 0;
    private double lastTime = 0;
    
    // A <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html">slew rate limiter</a> that limits the acceleration. Essentially, it caps the maximum rate-of-change
    private SlewRateLimiter limiter = new SlewRateLimiter(MAX_ACCELERATION / 10.0);

    /**
     * This command makes the elevator follow a target
     * 
     * The target is specified via a DoubleSupplier.
     * 
     * Usage Example:
     *  Command c = new ElevatorFollowCommand(elevatorSubsystem, () => Math.sin(++i % 50));
     *  // Do something with the command
     */
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
        double tgt = target.getAsDouble(); // The goal
        double dX = tgt - lastTarget; // How far we need to go
        double dt = Timer.getFPGATimestamp() - lastTime; // delta time

        // Set the elevator goal to the target value
        elevatorSub.setGoal(new TrapezoidProfile.State(
                MathUtil.clamp(limiter.calculate(tgt), 0,
                        elevatorHeightMeters),
                0.0));

        // Update values for the next dt and dX calculations
        lastTime = Timer.getFPGATimestamp();
        lastTarget = tgt;
    }

    @Override
    public void end(boolean interrupted) {
        // NOTE: Don't disable (so it position-holds with feedforward)
        // elevatorSub.disable();
    }
}
