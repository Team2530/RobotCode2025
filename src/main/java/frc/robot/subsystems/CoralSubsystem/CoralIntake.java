package frc.robot.subsystems.CoralSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Coral;

public class CoralIntake extends SubsystemBase {
    public final SparkMax intakeMotor = new SparkMax(Coral.Intake.MOTOR_PORT, MotorType.kBrushless);
    
    @Override
    public void periodic() {
        
    }
}
