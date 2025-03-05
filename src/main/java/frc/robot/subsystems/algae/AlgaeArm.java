package frc.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Algae;

@Logged
public class AlgaeArm extends SubsystemBase {
    private final SparkMax pivotMotor = new SparkMax(Algae.Pivot.MOTOR_PORT, MotorType.kBrushless);
    private RelativeEncoder pivotEncoder;
    // sim motor
    private final SparkMaxSim simPivotMotor = new SparkMaxSim(pivotMotor, Algae.Pivot.PhysicalConstants.MOTOR);

    // physics simulation
    private final SingleJointedArmSim simPivotPhysics = new SingleJointedArmSim(
            Algae.Pivot.PhysicalConstants.MOTOR,
            Algae.Pivot.PhysicalConstants.NET_REDUCTION,
            Algae.Pivot.PhysicalConstants.MOI,
            Algae.Pivot.PhysicalConstants.ARM_LENGTH_METERS,
            Algae.Pivot.RETRACTED_LIMIT,
            Algae.Pivot.EXTENDED_LIMIT,
            true,
            Algae.Pivot.RETRACTED_LIMIT);

    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    private final ProfiledPIDController pivotPID = Algae.Pivot.PID;
    // private final ArmFeedforward pivotFeedforward = Algae.Pivot.FEEDFORWARD;

    public AlgaeArm() {
        pivotConfig.idleMode(IdleMode.kBrake).inverted(Constants.Algae.Pivot.MOTOR_INVERTED).apply(
                new EncoderConfig()
                        .positionConversionFactor(
                                (2. * Math.PI) / Constants.Algae.Pivot.PhysicalConstants.NET_REDUCTION)
                        .velocityConversionFactor(
                                (2. * Math.PI) / (60.0 * Constants.Algae.Pivot.PhysicalConstants.NET_REDUCTION)));
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0.0f); // TODO: Add zeroing!

        if (Constants.Algae.DEBUG_PIDS) {
            SmartDashboard.putNumber("Algae/Pivot/PID/P", pivotPID.getP());
            SmartDashboard.putNumber("Algae/Pivot/PID/I", pivotPID.getI());
            SmartDashboard.putNumber("Algae/Pivot/PID/D", pivotPID.getD());
        }
    }

    @Override
    public void periodic() {
        if (Constants.Algae.DEBUG_PIDS) {
            pivotPID.setP(SmartDashboard.getNumber("Algae/Pivot/PID/P", pivotPID.getP()));
            pivotPID.setI(SmartDashboard.getNumber("Algae/Pivot/PID/I", pivotPID.getI()));
            pivotPID.setD(SmartDashboard.getNumber("Algae/Pivot/PID/D", pivotPID.getD()));
        }

        double output = pivotPID.calculate(pivotEncoder.getPosition());

        SmartDashboard.putNumber("Algae/Pivot/out", output);
        SmartDashboard.putNumber("Algae/Pivot/position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Algae/Pivot/target", pivotPID.getSetpoint().position);
        SmartDashboard.putNumber("Algae/Pivot/velocity_target", pivotPID.getSetpoint().velocity);
        SmartDashboard.putNumber("Algae/Pivot/velocity", pivotEncoder.getVelocity());
        SmartDashboard.putNumber("Algae/Pivot/goal", pivotPID.getGoal().position);

        pivotMotor.set(output);
        if (Robot.isSimulation())
            simPivotMotor.setAppliedOutput(output);
    }

    @Override
    public void simulationPeriodic() {
        // update physics
        simPivotPhysics.setInput(simPivotMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        simPivotPhysics.update(0.02);

        // update sim objects
        simPivotMotor.iterate(
                Units.radiansPerSecondToRotationsPerMinute(simPivotPhysics.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(),
                0.02);
    }

    public void setGoalDegrees(double goal) {
        pivotPID.setGoal(MathUtil.clamp(
                Units.degreesToRadians(goal),
                Algae.Pivot.RETRACTED_LIMIT,
                Algae.Pivot.EXTENDED_LIMIT));
    }

    @Logged
    public double getGoalDegrees() {
        return Units.radiansToDegrees(pivotPID.getGoal().position);
    }

    @Logged
    public double getPositionDegrees() {
        return Robot.isReal()
                ? Units.radiansToDegrees(pivotEncoder.getPosition())
                : Units.radiansToDegrees(simPivotPhysics.getAngleRads());
    }
}
