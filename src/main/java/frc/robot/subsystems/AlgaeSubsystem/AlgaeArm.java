package frc.robot.subsystems.AlgaeSubsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algae;

public class AlgaeArm extends SubsystemBase{
    public final SparkFlex pivotMotor = new SparkFlex(Algae.Pivot.MOTOR_PORT, MotorType.kBrushless);
    public final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    public final CANcoder pivotEncoder = new CANcoder(Algae.Pivot.ENCODER_PORT);

    public final ProfiledPIDController pivotPID = Algae.Pivot.PID;

    public AlgaeArm() {
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double output = pivotPID.calculate(pivotEncoder.getAbsolutePosition().getValueAsDouble());

        pivotMotor.set(output);
    }

    public void setGoalDegrees(double goal) {
        pivotPID.setGoal(MathUtil.clamp(goal, Algae.Pivot.RETRACTED_LIMIT_DEGREES, Algae.Pivot.EXTENDED_LIMIT_DEGREES));
    }

    public double getGoalDegrees() {
        return pivotPID.getGoal().position;
    }

    public double getPositionDegrees() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble();
    }
}
