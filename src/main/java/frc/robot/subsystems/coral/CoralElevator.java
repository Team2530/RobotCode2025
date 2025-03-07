package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

    LinearFilter curFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

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

    private ElevatorFeedforward feedForward = Elevator.FEEDFORWARD;
    private ProfiledPIDController elevatorPID = new ProfiledPIDController(
            Elevator.PID.kP,
            Elevator.PID.kI,
            Elevator.PID.kD,
            new TrapezoidProfile.Constraints(
                    Elevator.PID.MAX_VELOCITY,
                    Elevator.PID.MAX_ACCELERATION));

    public CoralElevator() {
        SoftLimitConfig limconfig = new SoftLimitConfig().reverseSoftLimit(0.03).reverseSoftLimitEnabled(true);
        // .forwardSoftLimit(Constants.Elevator.PhysicalParameters.MAX_TRAVEL).forwardSoftLimitEnabled(true);

        leaderConfig
                .idleMode(IdleMode.kBrake)
                .inverted(Elevator.Leader.INVERTED)
                .apply(limconfig);
        leaderConfig.encoder
                .positionConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER)
                .velocityConversionFactor(60.0 / Elevator.MOTOR_REVOLUTIONS_PER_METER); // Multiply by 60/RpM to get m/s

        followerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(Elevator.Follower.INVERTED)
                .apply(limconfig);

        followerConfig.encoder
                .positionConversionFactor(1 / Elevator.MOTOR_REVOLUTIONS_PER_METER)
                .velocityConversionFactor(60.0 / Elevator.MOTOR_REVOLUTIONS_PER_METER);

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderEncoder.setPosition(0);
        followerEncoder.setPosition(0);

        // Add some tolerance to the elevator controller
        elevatorPID.setTolerance(Units.inchesToMeters(0.5));

        if (Constants.Elevator.DEBUG_PIDS) {
            SmartDashboard.putNumber("Elevator/PID/P", elevatorPID.getP());
            SmartDashboard.putNumber("Elevator/PID/I", elevatorPID.getI());
            SmartDashboard.putNumber("Elevator/PID/D", elevatorPID.getD());
            SmartDashboard.putNumber("Elevator/FF/Kg", feedForward.getKg());
            SmartDashboard.putNumber("Elevator/FF/Ks", feedForward.getKs());
            SmartDashboard.putNumber("Elevator/FF/Kv", feedForward.getKv());
            SmartDashboard.putNumber("Elevator/FF/Ka", feedForward.getKa());
        }
    }

    private boolean isZeroed = true;

    private double lastVelocity = 0.0;
    private double lastTime = 0.0;

    private boolean testModeConfigured = false;

    @Override
    public void periodic() {
        if (Constants.Elevator.DEBUG_PIDS) {
            elevatorPID.setP(SmartDashboard.getNumber("Algae/Pivot/PID/P", elevatorPID.getP()));
            elevatorPID.setI(SmartDashboard.getNumber("Algae/Pivot/PID/I", elevatorPID.getI()));
            elevatorPID.setD(SmartDashboard.getNumber("Algae/Pivot/PID/D", elevatorPID.getD()));
            // feedForward = new ElevatorFeedforward(fee, lastVelocity, lastTime)
            double Kg = SmartDashboard.getNumber("Elevator/FF/Kg", feedForward.getKg());
            double Ks = SmartDashboard.getNumber("Elevator/FF/Ks", feedForward.getKs());
            double Kv = SmartDashboard.getNumber("Elevator/FF/Kv", feedForward.getKv());
            double Ka = SmartDashboard.getNumber("Elevator/FF/Ka", feedForward.getKa());

            feedForward = new ElevatorFeedforward(Ks, Kg, Kv, Ka);
        }

        if (DriverStation.isTest() && !testModeConfigured) {
            leaderMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            followerMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            testModeConfigured = true;
        } else if (!DriverStation.isTest() && testModeConfigured) {
            leaderMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            followerMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            testModeConfigured = false;
        }

        // check if needs to be zeroed and is at zero
        // TODO: ######################### PLACEHOLDERS AGAIN #########################
        // if (!isZeroed
        // && (curFilter.calculate((leaderMotor.getOutputCurrent() +
        // followerMotor.getOutputCurrent()) / 2) > 15
        // || Robot.isSimulation())) {
        // leaderEncoder.setPosition(0.0);
        // followerEncoder.setPosition(0.0);
        // isZeroed = true;
        // leaderMotor.configure(leaderConfig.apply(
        // new SoftLimitConfig().reverseSoftLimit(0.03).reverseSoftLimitEnabled(
        // true)),
        // ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // followerMotor.configure(leaderConfig.apply(
        // new SoftLimitConfig().reverseSoftLimit(0.03).reverseSoftLimitEnabled(
        // true)),
        // ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // }

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
                leaderMotor.set(-0.08);
                followerMotor.set(-0.08);
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

    // TODO: Fix!!!
    public void zeroElevator() {
        isZeroed = false;
        leaderMotor.configure(leaderConfig.apply(
                new SoftLimitConfig().reverseSoftLimit(0.03).reverseSoftLimitEnabled(
                        false)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        followerMotor.configure(leaderConfig.apply(
                new SoftLimitConfig().reverseSoftLimit(0.03).reverseSoftLimitEnabled(
                        false)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
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
}
