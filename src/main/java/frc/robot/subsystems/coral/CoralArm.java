package frc.robot.subsystems.coral;

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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
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
    private final ArmFeedforward pivotFeedforward = Coral.Pivot.FEEDFORWARD;
    private final ProfiledPIDController rollPID = Coral.Roll.PID;
    private final ProfiledPIDController pitchPID = Coral.Pitch.PID;
    

    private double pivotGoal = 0.0;
    private double rollGoal = 0.0;
    private double pitchGoal = 0.0;

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
        // if entering the frame border 
        // && wrist is at neutral
        if (
            (pivotPID.getGoal().position != pivotGoal)
            && (rollPID.atGoal() && pitchPID.atGoal())
        ) {
            // allow arm to enter frame border
            pivotPID.setGoal(pivotGoal);
        } else if ( // if exiting the frame border && has exited the frame border
            ((rollGoal != rollPID.getGoal().position) || (pitchGoal != pitchPID.getGoal().position))
            && (Math.abs(this.getPitchPositionDegrees()) > Coral.Pivot.FRAME_BORDER_ANGLE) 
        ) {
            // set roll and pitch back to their goals
            rollPID.setGoal(rollGoal);
            pitchPID.setGoal(pitchGoal);
        }

        // run the motors
        pivotMotor.set(
            pivotPID.calculate(pivotEncoder.getAbsolutePosition().getValueAsDouble())
            + pivotFeedforward.calculate(Units.degreesToRadians(pivotPID.getGoal().position), 0.0)
        );
        rollMotor.set(rollPID.calculate(rollEncoder.getPosition()));
        pitchMotor.set(pitchPID.calculate(pitchEncoder.getPosition()));
    } 

    // this should be relative to straight upwards.
    // i.e. 0 should be straight vertical,
    // 20 would be to the right, -20 to the left
    public void setPivotGoalDegrees(double goal) {
        pivotGoal = MathUtil.clamp(goal, Coral.Pivot.MAXIMUM_ANGLE, -1 * Coral.Pivot.MAXIMUM_ANGLE);
        // if passing through the frame border
        if (
            ((this.getPivotPositionDegrees() < 0) == (pivotGoal < 0)) 
            && (Math.abs(this.getPivotPositionDegrees()) < Coral.Pivot.FRAME_BORDER_ANGLE)
        ) {
            // set the pivot to stop at the frame border to allow the wrist to move neutral
            pivotPID.setGoal(
                Coral.Pivot.FRAME_BORDER_ANGLE
                * (pivotGoal < 0 ? -1 : 1)
            );
            rollPID.setGoal(0);
            pitchPID.setGoal(0);
        } else {
            pivotPID.setGoal(pivotGoal);
        }
    }

    // similarly, this ranges from -180 to 180
    public void setRollGoalDegrees(double goal) {
        rollGoal = goal;
        // if outside of frame border
        if (Math.abs(this.getPivotPositionDegrees()) > Coral.Pivot.FRAME_BORDER_ANGLE) {
            // start moving to goal
            rollPID.setGoal(rollGoal);
        }
    }

    public void setPitchGoalDegrees(double goal) {
        pitchGoal = MathUtil.clamp(goal, Coral.Pitch.MAXIMUM_ANGLE, -1 * Coral.Pitch.MAXIMUM_ANGLE);
        // if outside the frame border
        if (Math.abs(this.getPivotPositionDegrees()) > Coral.Pivot.FRAME_BORDER_ANGLE) {
            // start moving to goal
            pitchPID.setGoal(pitchGoal);
        }
    }

    public double getPivotGoalDegrees() {
        return pivotGoal;
    }

    public double getRollGoalDegrees() {
        return rollGoal;
    }

    public double getPitchGoalDegrees() {
        return pitchGoal;
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
