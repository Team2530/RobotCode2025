package frc.robot.subsystems.AlgaeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Main extends SubsystemBase{
    
    private final AlgaeArm arm;
    private final AlgaeIntake intake;

    public Main(AlgaeArm arm, AlgaeIntake intake) {
        this.arm = arm;
        this.intake = intake;
    }

    @Override
    public void periodic() {
        
    }
}
