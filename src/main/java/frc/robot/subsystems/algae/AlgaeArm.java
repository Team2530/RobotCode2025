package frc.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Algae;

public class AlgaeArm extends SubsystemBase {
    private final SparkFlex pivotMotor = new SparkFlex(Algae.Pivot.MOTOR_PORT, MotorType.kBrushless);
    // sim motor
    private final SparkFlexSim simPivotMotor = new SparkFlexSim(pivotMotor, Algae.Pivot.PhysicalConstants.MOTOR);

    public final CANcoder pivotEncoder = new CANcoder(Algae.Pivot.ENCODER_PORT);

    // physics simulation
    private final SingleJointedArmSim simPivotPhysics = new SingleJointedArmSim(
            Algae.Pivot.PhysicalConstants.MOTOR,
            Algae.Pivot.PhysicalConstants.GEARING,
            Algae.Pivot.PhysicalConstants.MOI,
            Algae.Pivot.PhysicalConstants.ARM_LENGTH_METERS,
            Units.degreesToRadians(Algae.Pivot.RETRACTED_LIMIT_DEGREES),
            Units.degreesToRadians(Algae.Pivot.EXTENDED_LIMIT_DEGREES),
            true,
            Units.degreesToRadians(Algae.Pivot.RETRACTED_LIMIT_DEGREES));

    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    private final ProfiledPIDController pivotPID = Algae.Pivot.PID;
    private final ArmFeedforward pivotFeedforward = Algae.Pivot.FEEDFORWARD;

    public AlgaeArm() {
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double output = pivotPID.calculate(
                pivotEncoder.getAbsolutePosition().getValueAsDouble()
                        + pivotFeedforward.calculate(Units.degreesToRadians(this.getGoalDegrees()), 0.0));

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
        pivotPID.setGoal(MathUtil.clamp(goal, Algae.Pivot.RETRACTED_LIMIT_DEGREES, Algae.Pivot.EXTENDED_LIMIT_DEGREES));
    }

    public double getGoalDegrees() {
        return pivotPID.getGoal().position;
    }

    public double getPositionDegrees() {
        return Robot.isReal()
                ? pivotEncoder.getAbsolutePosition().getValueAsDouble()
                : Units.radiansToDegrees(simPivotPhysics.getAngleRads());
    }
}
