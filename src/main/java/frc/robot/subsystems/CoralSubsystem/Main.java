package frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Main extends SubsystemBase {

    private final CoralArm arm;
    private final CoralIntake intake;

    public Main(CoralArm arm, CoralIntake intake) {
        this.arm = arm;
        this.intake = intake;
    }

    @Override
    public void periodic() {
        
    }
}
