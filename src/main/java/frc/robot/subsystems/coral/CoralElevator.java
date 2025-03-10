package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Elevator;

@Logged
public class CoralElevator extends SubsystemBase {

    private final SparkFlex leaderMotor = new SparkFlex(Elevator.Leader.MOTOR_PORT, MotorType.kBrushless);
    private final SparkFlex followerMotor = new SparkFlex(Elevator.Follower.MOTOR_PORT, MotorType.kBrushless);
    // sim motors
    private final SparkFlexSim simLeaderMotor = new SparkFlexSim(leaderMotor, DCMotor.getNeoVortex(1));
    private final SparkFlexSim simFollowerMotor = new SparkFlexSim(followerMotor, DCMotor.getNeoVortex(1));

    private final RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
    private final RelativeEncoder followerEncoder = followerMotor.getEncoder();
    // sim encoders
    private final SparkRelativeEncoderSim simLeaderEncoder = new SparkRelativeEncoderSim(leaderMotor);

    // physics simulations
    private final ElevatorSim simElevator = new ElevatorSim(
            Elevator.PhysicalParameters.MOTOR,
            Elevator.PhysicalParameters.GEARING,
            Elevator.PhysicalParameters.CARRIAGE_MASS_KG,
            Elevator.PhysicalParameters.DRIVE_RADIUS_METERS,
            0,
            Elevator.PhysicalParameters.MAX_TRAVEL,
            true,
            0);

    private final SparkFlexConfig leaderConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();

    private final ElevatorFeedforward feedForward = Elevator.FEEDFORWARD;
    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(
            Elevator.PID.kP,
            Elevator.PID.kI,
            Elevator.PID.kD,
            new TrapezoidProfile.Constraints(
                    Elevator.PID.MAX_VELOCITY,
                    Elevator.PID.MAX_ACCELERATION));

    public CoralElevator() {
        leaderConfig
                .idleMode(IdleMode.kBrake)
                .inverted(Elevator.Leader.INVERTED);
        leaderConfig.encoder
                .positionConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER)
                .velocityConversionFactor(60.0 / Elevator.MOTOR_REVOLUTIONS_PER_METER); // Multiply by 60/RpM to get m/s

        followerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(Elevator.Follower.INVERTED);
        followerConfig.encoder
                .positionConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER)
                .velocityConversionFactor(60.0 / Elevator.MOTOR_REVOLUTIONS_PER_METER);

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderEncoder.setPosition(0);
        followerEncoder.setPosition(0);

        // Add some tolerance to the elevator controller
        elevatorPID.setTolerance(Units.inchesToMeters(0.5));
    }

    private boolean isZeroed = false;

    private double lastVelocity = 0.0;
    private double lastTime = 0.0;

    @Override
    public void periodic() {
        // check if needs to be zeroed and is at zero
        // TODO: ######################### PLACEHOLDERS AGAIN #########################
        if (!isZeroed
                && (((leaderMotor.getOutputCurrent() + followerMotor.getOutputCurrent()) / 2) > 20
                        || Robot.isSimulation())) {
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

            double pid_out = elevatorPID.calculate(getPosition());
            double ff_out = feedForward.calculate(setpoint.velocity, deltaVelocity / deltaTime);
            double output = ff_out + pid_out;

            SmartDashboard.putNumber("Elevator/output", output);
            SmartDashboard.putNumber("Elevator/position", getPosition());
            SmartDashboard.putNumber("Elevator/target", setpoint.position);
            SmartDashboard.putNumber("Elevator/goal", elevatorPID.getGoal().position);
            SmartDashboard.putNumber("Elevator/pid_out", pid_out);
            SmartDashboard.putNumber("Elevator/ff_out", ff_out);

            if (!Constants.Elevator.DBG_DISABLED) {
                leaderMotor.setVoltage(output);
                followerMotor.setVoltage(output);
            }

            if (Robot.isSimulation()) {
                simLeaderMotor.setAppliedOutput(output);
                simFollowerMotor.setAppliedOutput(output);
            }
        } else {
            if (!Constants.Elevator.DBG_DISABLED) {
                // slowly move down to zero
                leaderMotor.set(-0.05);
                followerMotor.set(-0.05);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // update physics
        simElevator
                .setInput(MathUtil.clamp(simLeaderMotor.getAppliedOutput() * RoboRioSim.getVInVoltage(), -12.0, 12.0));
        simElevator.update(0.02);

        // update sim objects
        simLeaderMotor.iterate(
                simElevator.getVelocityMetersPerSecond(),
                RoboRioSim.getVInVoltage(),
                0.02);
        simFollowerMotor.iterate(
                simElevator.getVelocityMetersPerSecond(),
                RoboRioSim.getVInVoltage(),
                0.02);
    }

    public void setGoalPosition(double position) {
        elevatorPID.setGoal(MathUtil.clamp(position, 0.0, Elevator.PhysicalParameters.MAX_TRAVEL));
    }

    public double getGoalPosition() {
        return elevatorPID.getGoal().position;
    }

    public double getPosition() {
        return Robot.isReal()
                ? leaderEncoder.getPosition()
                : simElevator.getPositionMeters();
    }

    public boolean isInPosition() {
        return elevatorPID.atGoal();
    }

    public boolean isSupposedToBeInPosition() {
        return MathUtil.isNear(elevatorPID.getSetpoint().position, elevatorPID.getGoal().position,
                Units.inchesToMeters(1 / 16.0));
    }

    public void zeroElevator() {
        isZeroed = false;
    }
}
