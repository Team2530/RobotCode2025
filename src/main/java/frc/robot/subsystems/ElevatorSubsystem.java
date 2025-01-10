package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorSubsystem extends ProfiledPIDSubsystem {
    // NOTE: Elevator motor one has both the encoder used for positioning, and the
    // limit switch used for zeroing
    private final SparkFlex elevatorMotorOne = new SparkFlex(Constants.Elevator.elevatorOnePort,
            MotorType.kBrushless);
    private final SparkFlex elevatorMotorTwo = new SparkFlex(Constants.Elevator.elevatorTwoPort,
            MotorType.kBrushless);
    private final ElevatorFeedforward feedForward = new ElevatorFeedforward(
            Constants.Elevator.Feedforward.Ks,
            Constants.Elevator.Feedforward.Kg,
            Constants.Elevator.Feedforward.Kv,
            Constants.Elevator.Feedforward.Ka

    );

    private final SparkMaxConfig elevatorConfigOne = new SparkMaxConfig();
    private final SparkMaxConfig elevatorConfigTwo = new SparkMaxConfig();

    /*
     * DCMotor gearbox,
     * double gearing,
     * double carriageMassKg,
     * double drumRadiusMeters,
     * double minHeightMeters,
     * double maxHeightMeters,
     * boolean simulateGravity,
     * double startingHeightMeters,
     * Matrix<N1, N1> measurementStdDevs
     */
    private boolean isZeroed = false;
    private final RelativeEncoder elevatorEncoderOne;
    private final RelativeEncoder elevatorEncoderTwo;
    private final SparkLimitSwitch bottomLimit;

    private final ElevatorSim simulation = new ElevatorSim(
            Constants.Elevator.PhysicalParameters.simMotor,
            Constants.Elevator.PhysicalParameters.gearReduction,
            Constants.Elevator.PhysicalParameters.carriageMassKg,
            Constants.Elevator.PhysicalParameters.driveRadiusMeters,
            0.0,
            Constants.Elevator.PhysicalParameters.elevatorHeightMeters,
            true,
            0.0,
            0.001,
            0.001
    );

    private DoubleLogEntry elevatorTargetP = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/target/position");
    private DoubleLogEntry elevatorTargetV = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/target/velocity");
    private DoubleLogEntry elevatorP = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/state/position");
    private DoubleLogEntry elevatorV = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/state/velocity");
    private DoubleLogEntry elevatorOneOutput = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/output1");
    private DoubleLogEntry elevatorTwoOutput = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/output2");
    private DoubleLogEntry elevatorOneCurrent = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/current1");
    private DoubleLogEntry elevatorTwoCurrent = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/current2");

    private Mechanism2d mechanism2D = new Mechanism2d(Constants.RobotConstants.robotLengthMeters,
            Constants.Elevator.PhysicalParameters.elevatorBottomFromFloorMeters
                    + Constants.Elevator.PhysicalParameters.elevatorHeightMeters
                    + Constants.Elevator.PhysicalParameters.elevatorCarriageHeightMeters / 2.0);
    private MechanismRoot2d rootMechanism = mechanism2D.getRoot("climber",
            Constants.RobotConstants.robotLengthMeters / 2.0
                    + Constants.Elevator.PhysicalParameters.elevatorForwardsFromRobotCenterMeters,
            Constants.Elevator.PhysicalParameters.elevatorBottomFromFloorMeters);
    private MechanismLigament2d elevatorMechanism;

    public ElevatorSubsystem() {
        super(
                new ProfiledPIDController(
                        Constants.Elevator.PID.kP,
                        Constants.Elevator.PID.kI,
                        Constants.Elevator.PID.kD,
                        new TrapezoidProfile.Constraints(
                                Constants.Elevator.PID.MAX_VELOCITY,
                                Constants.Elevator.PID.MAX_ACCELERATION)),
                0.0);

        this.getController().setTolerance(Units.inchesToMeters(0.5));

        elevatorConfigOne
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.Elevator.elevatorOneInverted);
        elevatorConfigOne.encoder
            .positionConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter)
            .velocityConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);
        elevatorConfigOne.limitSwitch.reverseLimitSwitchType(Constants.Elevator.bottomLimitMode);

        elevatorConfigTwo
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.Elevator.elevatorTwoInverted);
        elevatorConfigTwo.encoder
            .positionConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter)
            .velocityConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);

        elevatorMotorOne.configure(elevatorConfigOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorTwo.configure(elevatorConfigTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        elevatorEncoderOne = elevatorMotorOne.getEncoder();
        elevatorEncoderTwo = elevatorMotorTwo.getEncoder();

        elevatorMotorOne.getEncoder().setPosition(0);
        elevatorMotorTwo.getEncoder().setPosition(0);

        bottomLimit = elevatorMotorOne.getReverseLimitSwitch();

        this.enable();

        this.elevatorMechanism = rootMechanism.append(new MechanismLigament2d("elevator", 0.0, 90));
    }

    public void setPosition(double positionMeters) {
        setGoal(MathUtil.clamp(positionMeters, 0.0, Constants.Elevator.PhysicalParameters.elevatorHeightMeters));
    }

    public double getPosition() {
        return getMeasurement();
    }

    public double getGoalPosition() {
        return this.getController().getGoal().position;
    }

    double lastVelocity = 0.0;
    double lastTime = 0.0;

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double dv = setpoint.velocity - lastVelocity;
        double dt = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();

        lastVelocity = setpoint.velocity;

        double ff = feedForward.calculate(setpoint.velocity, dv / dt);

        if (isZeroed || Robot.isSimulation()) {
            // Controller output voltage
            double op = ff + output;

            elevatorMotorOne.setVoltage(op);
            elevatorMotorTwo.setVoltage(op);

            elevatorTargetP.append(setpoint.position);
            elevatorTargetV.append(setpoint.velocity);

            SmartDashboard.putNumber("Elevator/Current Position", getMeasurement());
            SmartDashboard.putNumber("Elevator/Current Velocity", elevatorEncoderOne.getVelocity());

            SmartDashboard.putNumber("Elevator/Target Position", setpoint.position);
            SmartDashboard.putNumber("Elevator/Target Velocity", setpoint.velocity);

            if (Robot.isSimulation()) {
                simulation.setInputVoltage(MathUtil.clamp((output + ff), -12.0, 12.0));
            }
        } else {
            // Move down a little bit to zero
            elevatorMotorOne.set(-0.05);
            elevatorMotorTwo.set(-0.05);
        }
    }

    @Override
    public double getMeasurement() {
        return Robot.isSimulation() ? simulation.getPositionMeters() : elevatorEncoderOne.getPosition();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub

        if ((bottomLimit.isPressed()) && (!isZeroed)) {
            elevatorEncoderOne.setPosition(0.0);
            setGoal(0.0);
            isZeroed = true;
        }

        super.periodic();

        elevatorP.append(getMeasurement());
        elevatorV.append(elevatorEncoderOne.getVelocity());
        elevatorOneOutput.append(elevatorMotorOne.getAppliedOutput());
        elevatorTwoOutput.append(elevatorMotorTwo.getAppliedOutput());
        elevatorOneCurrent.append(elevatorMotorOne.getOutputCurrent());
        elevatorTwoCurrent.append(elevatorMotorTwo.getOutputCurrent());
        this.elevatorMechanism
                .setLength(getMeasurement() + Constants.Elevator.PhysicalParameters.elevatorCarriageHeightMeters / 2.0);

        // SmartDashboard.putNumber("Current Elevator Position", getMeasurement());
        // SmartDashboard.putNumber("Goal Elevator Position",
        // this.getController().getGoal().position);
        SmartDashboard.putBoolean("Elevator Bottom Limit", bottomLimit.isPressed());
        SmartDashboard.putData("Elevator Mechanism", mechanism2D);
    }

    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        super.simulationPeriodic();

        simulation.update(0.020);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simulation.getCurrentDrawAmps()));
    }

    public boolean isInPosition() {
        return this.getController().atGoal();
    }
}