package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class MotionSequencer {
    private final Arm arm;
    private final Wrist wrist;

    public enum Joints {
        THETA,
        PHI,
        ARM;
    }

    public MotionSequencer(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;
    }

    public Command stowCoral() {
        return new ParallelCommandGroup(
            //TODO: Add functionality
        );
    }

    public Command deployCoral() {
        return new SequentialCommandGroup(
            //TODO: Add functionality
        );
    }

    public boolean canArticulate(Joints joint) {
        switch (joint) {
            case ARM:
                return Math.abs(wrist.getPhiPosition()) < 1.5 && Math.abs(wrist.getThetaPosition()) < 1.5;
            case PHI:
                return Math.abs(arm.getPosition()) > 50 && Math.abs(arm.getPosition()) < 140;
            case THETA:
                return Math.abs(arm.getPosition()) > 50 && Math.abs(arm.getPosition()) < 140;
        }

        return false;
    }

    private BooleanSupplier isClear(Joints joint) {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return canArticulate(joint);
            }
        };
    }
}
