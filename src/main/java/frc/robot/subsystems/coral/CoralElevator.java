package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

@Logged(strategy = Strategy.OPT_IN)
public class CoralElevator extends SubsystemBase {
    private final TalonFX leader;
    private final TalonFX follower;

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    MotionMagicVoltage mmReq = new MotionMagicVoltage(0.0);

    @Logged
    double positionGoal = 0.0;
    @Logged
    double currentPosition = 0.0;
    // @Logged
    // double currentTarget = 0.0;

    private final ElevatorSim simElevator = new ElevatorSim(
            Elevator.PhysicalParameters.MOTOR,
            Elevator.PhysicalParameters.GEARING,
            Elevator.PhysicalParameters.CARRIAGE_MASS_KG,
            Elevator.PhysicalParameters.DRIVE_RADIUS_METERS,
            0,
            Elevator.PhysicalParameters.MAX_TRAVEL,
            true,
            0);

    public CoralElevator() {
        leader = new TalonFX(Constants.Elevator.Leader.MOTOR_PORT);
        follower = new TalonFX(Constants.Elevator.Follower.MOTOR_PORT);

        FeedbackConfigs fbc = elevatorConfig.Feedback;
        fbc.SensorToMechanismRatio = Constants.Elevator.MOTOR_REVOLUTIONS_PER_METER;

        MotionMagicConfigs mmc = elevatorConfig.MotionMagic;
        mmc.MotionMagicAcceleration = Constants.Elevator.PID.MAX_ACCELERATION;
        mmc.MotionMagicCruiseVelocity = Constants.Elevator.PID.MAX_VELOCITY;
        mmc.MotionMagicJerk = Constants.Elevator.PID.MAX_JERK;

        Slot0Configs s0c = elevatorConfig.Slot0;
        s0c.GravityType = GravityTypeValue.Elevator_Static;
        s0c.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        s0c.kA = Constants.Elevator.FeedforwardConstants.Ka;
        s0c.kV = Constants.Elevator.FeedforwardConstants.Kv;
        s0c.kS = Constants.Elevator.FeedforwardConstants.Ks;
        s0c.kG = Constants.Elevator.FeedforwardConstants.Kg;

        s0c.kP = Constants.Elevator.PID.kP;
        s0c.kI = Constants.Elevator.PID.kI;
        s0c.kD = Constants.Elevator.PID.kD;

        leader.getConfigurator().apply(elevatorConfig);
        leader.getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(Constants.Elevator.Leader.INVERTED ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive));

        follower.setControl(new Follower(Constants.Elevator.Leader.MOTOR_PORT, true));

        leader.setPosition(0.0);
        follower.setPosition(0.0);

        // Soft limits
        leader.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs().withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(0.03));
        follower.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs().withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(0.03));
    }

    @Override
    public void periodic() {
        leader.setControl(mmReq.withPosition(positionGoal).withSlot(0));
        currentPosition = leader.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Elevator/goal", positionGoal);
        SmartDashboard.putNumber("Elevator/position", currentPosition);
    }

    @Override
    public void simulationPeriodic() {
        var leaderSimState = leader.getSimState();
        leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        simElevator.setInputVoltage(leaderSimState.getMotorVoltage());
        simElevator.update(0.02);
        leaderSimState
                .setRawRotorPosition(simElevator.getPositionMeters() * Constants.Elevator.MOTOR_REVOLUTIONS_PER_METER);
        leaderSimState.setRotorVelocity(
                simElevator.getVelocityMetersPerSecond() * Constants.Elevator.MOTOR_REVOLUTIONS_PER_METER);
    }

    public void setGoalPosition(double targetMeters) {
        positionGoal = targetMeters;
    }

    public double getPosition() {
        return leader.getPosition().getValueAsDouble();
    }

    public double getGoalPosition() {
        return positionGoal;
    }

    public boolean isInPosition() {
        return MathUtil.isNear(getPosition(), getGoalPosition(), Units.inchesToMeters(0.75));
    }

    // TODO: Add zeroing!!!
}
