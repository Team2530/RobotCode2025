package frc.robot.subsystems.CoralSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Coral;

public class CoralIntake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(Coral.Intake.MOTOR_PORT, MotorType.kBrushless);
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(
        Coral.Intake.POSITIVE_RATE_LIMIT,
        Coral.Intake.NEGATIVE_RATE_LIMIT,
        0
    );

    public CoralIntake() {
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private double outputPercentage = 0.0;

    @Override
    public void periodic() {
        double output = intakeProfile.calculate(outputPercentage);

        intakeMotor.set(output);
    }

    public void setOutputPercentage(double outputPercentage) {
        this.outputPercentage = MathUtil.clamp(outputPercentage, -1, 1);
    }

    public double getOutputPercentage() {
        return outputPercentage;
    }
}
