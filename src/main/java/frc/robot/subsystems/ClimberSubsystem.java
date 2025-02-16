package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(Climber.MOTOR_PORT, MotorType.kBrushless);
    private SparkMaxConfig climberConfig = new SparkMaxConfig();
    private RelativeEncoder climberEncoder;
    XboxController operator;

    public ClimberSubsystem(XboxController operator) {
        this.operator = operator;
        climberMotor.configure(climberConfig.idleMode(IdleMode.kBrake)
                // .apply(new
                // SoftLimitConfig().forwardSoftLimit(Constants.Climber.DEPLOY_SOFT_LIMIT)
                // .forwardSoftLimitEnabled(true).reverseSoftLimit(Constants.Climber.CLIMB_SOFT_LIMIT))
                .apply(
                        new EncoderConfig().positionConversionFactor(1.0 / Constants.Climber.GEAR_RATIO)
                                .velocityConversionFactor(
                                        1.0 / Constants.Climber.GEAR_RATIO)),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        climberEncoder = climberMotor.getEncoder();
    }

    private double output = 0.0;
    private boolean deployed = false;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", climberEncoder.getPosition());
        // if (climberEncoder.getPosition() > Constants.Climber.CLIMB_SOFT_LIMIT &&
        // !deployed) {
        // deployed = true;
        // climberMotor.configure(climberConfig.apply(climberConfig.softLimit.reverseSoftLimitEnabled(true)),
        // ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // }

        if (!Constants.Climber.DBG_DISABLED)
            climberMotor.set(operator.getLeftBumper() ? (operator.getLeftY()) : 0.0);
    }

    public void setOutput(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }
}
