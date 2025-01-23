package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax leaderMotor = new SparkMax(Elevator.Leader.MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax followerMotor = new SparkMax(Elevator.Follower.MOTOR_PORT, MotorType.kBrushless);

    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    private final RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
    private final RelativeEncoder followerEncoder = followerMotor.getEncoder();

    private final SparkLimitSwitch bottomLimit = leaderMotor.getReverseLimitSwitch();


    private final ElevatorFeedforward feedForward = Elevator.FEEDFORWARD;
    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(
        Elevator.PID.kP,
        Elevator.PID.kI,
        Elevator.PID.kD,
        new TrapezoidProfile.Constraints(
            Elevator.PID.MAX_VELOCITY, 
            Elevator.PID.MAX_ACCELERATION
        )
    );

    public ElevatorSubsystem() {
        leaderConfig
            .idleMode(IdleMode.kBrake)
            .inverted(Elevator.Leader.INVERTED);  
        leaderConfig.encoder
            .positionConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER)
            .velocityConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER);
            
        followerConfig
            .idleMode(IdleMode.kBrake)
            .inverted(Elevator.Follower.INVERTED);
        followerConfig.encoder
            .positionConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER)
            .velocityConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER);

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderEncoder.setPosition(0);
        followerEncoder.setPosition(0);
    }


    private boolean isZeroed = false;

    private double lastVelocity = 0.0;
    private double lastTime = 0.0;

    @Override
    public void periodic() {
        // check if needs to be zeroed and is at zero
        if (!isZeroed && bottomLimit.isPressed()) {
            leaderEncoder.setPosition(0);
            isZeroed = true;
        }

        // check if initial zero had been run
        if (isZeroed) {
            TrapezoidProfile.State setpoint = elevatorPID.getSetpoint();
            double deltaVelocity = setpoint.velocity - lastVelocity;
            double deltaTime = Timer.getFPGATimestamp() - lastTime;

            lastTime = Timer.getFPGATimestamp();
            lastVelocity = setpoint.velocity;

            double output = feedForward.calculate(setpoint.velocity, deltaVelocity / deltaTime)
                + elevatorPID.calculate(leaderEncoder.getPosition());

            leaderMotor.setVoltage(output);
            followerMotor.setVoltage(output);
        } else {
            // slowly move down to zero
            leaderMotor.set(-0.05);
            followerMotor.set(-0.05);
        }
    }

    public void setPosition(double position) {
        elevatorPID.setGoal(MathUtil.clamp(position, 0.0, Elevator.PhysicalParameters.elevatorHeightMeters));
    }

    public double getPosition() {
        return leaderEncoder.getPosition();
    }

    public double getGoalPosition() {
        return elevatorPID.getGoal().position;
    }

    public boolean isInPosition() {
        return elevatorPID.atGoal();
    }
}
