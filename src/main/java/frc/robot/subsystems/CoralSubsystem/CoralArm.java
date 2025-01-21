package frc.robot.subsystems.CoralSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Coral;

public class CoralArm extends SubsystemBase{
    private final SparkMax pivotMotor = new SparkMax(Coral.Pivot.MOTOR_PORT, MotorType.kBrushless);

    private final SparkMax rollMotor = new SparkMax(Coral.Roll.MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax pitchMotor = new SparkMax(Coral.Pitch.MOTOR_PORT, MotorType.kBrushless);

    @Override
    public void periodic() {

    }
}
