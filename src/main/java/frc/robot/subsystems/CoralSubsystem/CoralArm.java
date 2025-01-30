package frc.robot.subsystems.CoralSubsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.Constants.Coral;

public class CoralArm extends SubsystemBase{
    private final SparkFlex pivotMotor = new SparkFlex(Coral.Pivot.MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax rollMotor = new SparkMax(Coral.Roll.MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax pitchMotor = new SparkMax(Coral.Pitch.MOTOR_PORT, MotorType.kBrushless);

    private final CANcoder pivotEncoder = new CANcoder(Coral.Pivot.ENCODER_PORT);
    private final AbsoluteEncoder rollEncoder = rollMotor.getAbsoluteEncoder();
    private final AbsoluteEncoder pitchEncoder = pitchMotor.getAbsoluteEncoder();

    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private final SparkMaxConfig rollConfig = new SparkMaxConfig();
    private final SparkMaxConfig pitchConfig = new SparkMaxConfig();


    private final ProfiledPIDController pivotPID = Coral.Pivot.PID;
    private final ProfiledPIDController rollPID = Coral.Roll.PID;
    private final ProfiledPIDController pitchPID = Coral.Pitch.PID;
    

    public CoralArm() {
        pivotConfig.idleMode(IdleMode.kBrake);
        rollConfig.idleMode(IdleMode.kBrake);
        pitchConfig.idleMode(IdleMode.kBrake);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollMotor.configure(rollConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pitchMotor.configure(pitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void periodic() {
        pivotMotor.set(pivotPID.calculate(pivotEncoder.getAbsolutePosition().getValueAsDouble()));
        rollMotor.set(rollPID.calculate(rollEncoder.getPosition()));
        pitchMotor.set(pitchPID.calculate(pitchEncoder.getPosition()));
    }

    public void setPivotGoalDegrees(double goal) {
        pivotPID.setGoal(MathUtil.clamp(goal, Coral.Pivot.MAXIMUM_ANGLE, -1 * Coral.Pivot.MAXIMUM_ANGLE));
    }

    public void setRollGoalDegrees(double goal) {
        rollPID.setGoal(goal);
    }

    public void setPitchGoalDegress(double goal) {
        pitchPID.setGoal(MathUtil.clamp(goal, Coral.Pitch.MAXIMUM_ANGLE, -1 * Coral.Pitch.MAXIMUM_ANGLE));
    }

    public double getPivotGoalDegress() {
        return pivotPID.getGoal().position;
    }

    public double getRollGoalDegrees() {
        return rollPID.getGoal().position;
    }

    public double getPitchGoalDegrees() {
        return pitchPID.getGoal().position;
    }

    public double getPivotPositionDegrees() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getRollPositionDegrees() {
        return rollEncoder.getPosition();
    }

    public double getPitchPositionDegrees() {
        return pitchEncoder.getPosition();
    }

}
