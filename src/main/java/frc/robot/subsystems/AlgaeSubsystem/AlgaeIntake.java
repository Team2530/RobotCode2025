package frc.robot.subsystems.AlgaeSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algae;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(Algae.Intake.MOTOR_PORT, MotorType.kBrushless);

    @Override
    public void periodic() {

    }
}
