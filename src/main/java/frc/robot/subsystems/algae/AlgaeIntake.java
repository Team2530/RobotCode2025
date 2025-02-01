package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algae;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(Algae.Intake.MOTOR_PORT, MotorType.kBrushless);

    private final DigitalInput intakeBeambreak = new DigitalInput(Algae.Intake.BEAMBREAK_PORT);

    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(
        Algae.Intake.POSITIVE_RATE_LIMIT,
        Algae.Intake.NEGATIVE_RATE_LIMIT, 
        0
    );

    private double outputPercentage = 0.0;

    @Override
    public void periodic() {
        // if no algae
        if (intakeBeambreak.get()) {
            double output = intakeProfile.calculate(outputPercentage);
            intakeMotor.set(output);
        } else {
            intakeMotor.set(0.05);
        }

    }

    public void setOutputPercentage(double outputPercentage) {
        this.outputPercentage = MathUtil.clamp(outputPercentage, -1, 1);
    }

    public double getOutputPercentage() {
        return outputPercentage;
    }
}
