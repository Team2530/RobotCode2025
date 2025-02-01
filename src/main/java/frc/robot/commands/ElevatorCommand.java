package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.Elevator.PhysicalParameters.elevatorHeightMeters;

/**
 * This command tells the elevator to go to a preset.
 */
public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSub; // A reference to the elevator subsystem

    private final ElevatorPreset target; // The current selected target
    private final double offset; // An offset for the current target

    /**
     * Create a new instance of the command.
     * 
     * @param elevatorSystem The elevator subsystem
     * @param targetPosition The target position - one of 'STOW', 'MIDDLE' or 'TOP'. See the {@link ElevatorCommand.ElevatorPresets} enum for more info.
     * @param offset An offset for the target position
     */
    public ElevatorCommand(ElevatorSubsystem elevatorSystem, ElevatorPreset targetPosition, double targetOffset) {
        this.elevatorSub = elevatorSystem;
        this.target = targetPosition;
        this.offset = targetOffset;

        addRequirements(elevatorSub);
    }

    @Override
    public void initialize() {
        double tgt = target.getPosition() + offset; // The actual target position, in meters
        
        // Set the goal of the elevator subsystem after clamping it to a reasonable value.
        elevatorSub.setGoal(MathUtil.clamp(tgt, 0, elevatorHeightMeters));

        // Put it on smartdashboard
        SmartDashboard.putString("Elevator Command", target.toString());
    }

    /**
     * Check if the elevator is in its goal position.
     * 
     * @return If the elevator is in the current 'goal' position
     */
    @Override
    public boolean isFinished() {
        return elevatorSub.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        // NOTE: Don't disable (so it position-holds with feedforward)
        // elevatorSub.disable();
    }

    /**
     * Various presets for the elevator
     */
    public enum ElevatorPreset {
        STOW(0.0),
        MIDDLE(elevatorHeightMeters / 2),
        TOP(elevatorHeightMeters);

        // The position of the preset in meters
        private final double position;

        private ElevatorPresets(double pos_meters) {
            this.position = pos_meters;
        }

        /**
         * Returns the position of the elevator preset in meters
         */
        public double getPosition() {
            return this.position;
        }
    }
}
