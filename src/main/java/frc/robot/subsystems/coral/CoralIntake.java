package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Coral;

public class CoralIntake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(Coral.Intake.MOTOR_PORT);
    // sim motor
    private TalonFXSimState simIntakeMotor;

    // physics simulation
    private final FlywheelSim simIntakePhysics = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    Coral.Intake.PhysicalConstants.MOTOR,
                    Coral.Intake.PhysicalConstants.MOI,
                    Coral.Intake.PhysicalConstants.GEARING),
            Coral.Intake.PhysicalConstants.MOTOR);

    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(
            Coral.Intake.POSITIVE_RATE_LIMIT,
            -Coral.Intake.NEGATIVE_RATE_LIMIT,
            0);

    public CoralIntake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(20.0));
        intakeMotor.getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(Constants.Coral.Intake.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive));
    }

    private double outputPercentage = 0.0;
    // private boolean lastHolding = false;

    @Override
    public void periodic() {
        boolean curHolding = isHolding();
        // if no coral
        if (!curHolding) {
            double output = intakeProfile.calculate(outputPercentage);
            if (!Constants.Coral.Intake.DBG_DISABLED)
                intakeMotor.set(output);
        } else {
            // hold, negative is out so intake a bit.
            // Set to 0.1 to be good
            if (!Constants.Coral.Intake.DBG_DISABLED)
                intakeMotor.set(Math.min(0.1, outputPercentage));
        }

        // if (lastHolding != curHolding) {
        // intakeMotor.getConfigurator().apply(new
        // CurrentLimitsConfigs().withStatorCurrentLimit(
        // (curHolding && (outputPercentage > 0.0)) ?
        // Constants.Coral.Intake.HOLD_CURRENT_LIMIT
        // : Constants.Coral.Intake.IN_OUT_CURRENT_LIMIT));
        // }

        // lastHolding = curHolding;
    }

    @Override
    public void simulationPeriodic() {
        simIntakeMotor = intakeMotor.getSimState();
        simIntakeMotor.setSupplyVoltage(RoboRioSim.getVInVoltage());

        // update physics
        simIntakePhysics.setInput(simIntakeMotor.getMotorVoltage() * RoboRioSim.getVInVoltage());
        simIntakePhysics.update(0.02);

        // update sim objects
        // rpm to rps
        simIntakeMotor.setRotorVelocity(simIntakePhysics.getAngularVelocityRPM() / 60);
    }

    public void setOutputPercentage(double outputPercentage) {
        this.outputPercentage = MathUtil.clamp(outputPercentage, -1, 1);
    }

    public double getOutputPercentage() {
        return outputPercentage;
    }

    public boolean isHolding() {
        return Robot.isReal()
                ? intakeMotor.getForwardLimit().getValue().value == 0
                : false;
    }
}
