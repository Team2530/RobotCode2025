package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
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
    private final TalonFX intakeMotor = new TalonFX(Coral.Intake.MOTOR_PORT);

    private final StatusSignal<ReverseLimitValue> intakeBeambreak = intakeMotor.getReverseLimit();

    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(
        Coral.Intake.POSITIVE_RATE_LIMIT,
        Coral.Intake.NEGATIVE_RATE_LIMIT,
        0
    );

    public CoralIntake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private double outputPercentage = 0.0;

    @Override
    public void periodic() {
        // if no coral 
        if (intakeBeambreak.getValue().value == 1) {
            double output = intakeProfile.calculate(outputPercentage);
            intakeMotor.set(output);
        } else {
            // hold
            intakeMotor.set(Math.min(0.05, outputPercentage));
        }

    }

    public void setOutputPercentage(double outputPercentage) {
        this.outputPercentage = MathUtil.clamp(outputPercentage, -1, 1);
    }

    public double getOutputPercentage() {
        return outputPercentage;
    }
}
