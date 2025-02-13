package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(Climber.MOTOR_PORT, MotorType.kBrushless);
    private final SparkMaxConfig climberConfig = new SparkMaxConfig();

    public ClimberSubsystem() {
        climberConfig.idleMode(IdleMode.kBrake);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private double output = 0.0;

    @Override
    public void periodic() {
        if (!Constants.Climber.DBG_DISABLED)
            climberMotor.set(output);
    }

    // TODO: limit the output somehow
    public void setOutput(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }
}
