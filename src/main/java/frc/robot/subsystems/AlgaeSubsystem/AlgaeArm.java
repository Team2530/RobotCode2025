package frc.robot.subsystems.AlgaeSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algae;

public class AlgaeArm extends SubsystemBase{
    public final SparkMax pivotMotor = new SparkMax(Algae.Pivot.MOTOR_PORT, MotorType.kBrushless);
        
    @Override
    public void periodic() {
        
    }
}
