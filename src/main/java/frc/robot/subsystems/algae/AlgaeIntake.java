package frc.robot.subsystems.algae;

import java.util.function.BooleanSupplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algae;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeIntakePresets;
import frc.robot.Constants;
import frc.robot.Robot;

@Logged
public class AlgaeIntake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(Algae.Intake.MOTOR_PORT, MotorType.kBrushless);
    private SparkBaseConfig intakeMotorConfig = new SparkMaxConfig();
    // sim motor
    private final SparkMaxSim simIntakeMotor = new SparkMaxSim(intakeMotor, Algae.Intake.PhysicalConstants.MOTOR);

    // physics simulation
    private final FlywheelSim simIntakePhysics = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    Algae.Intake.PhysicalConstants.MOTOR,
                    Algae.Intake.PhysicalConstants.MOI,
                    Algae.Intake.PhysicalConstants.GEARING),
            Algae.Intake.PhysicalConstants.MOTOR);

    // private final DigitalInput intakeBeambreak = new
    // DigitalInput(Algae.Intake.BEAMBREAK_PORT);
    private final LaserCan intakeSensor = new LaserCan(Constants.Algae.Intake.LASERCAN_ID);

    private AlgaeIntakePresets currentPreset = AlgaeIntakePresets.STOP;

    private boolean holding_internal = false;

    public AlgaeIntake() {
        intakeMotorConfig = intakeMotorConfig.inverted(Constants.Algae.Intake.MOTOR_INVERTED);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        intakeMotor.set(currentPreset.outputPercentage);
        if (Robot.isSimulation())
            simIntakeMotor.setAppliedOutput(currentPreset.outputPercentage);

        if (currentPreset == AlgaeIntakePresets.HOLD || currentPreset == AlgaeIntakePresets.INTAKING) {
            holding_internal = getSensorDistance() < Constants.Algae.Intake.HOLDING_THRESHOLD;
        } else {
            holding_internal = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // update physics
        simIntakePhysics.setInput(simIntakeMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        simIntakePhysics.update(0.02);

        // update sim objects
        simIntakeMotor.iterate(
                simIntakePhysics.getAngularVelocityRPM(),
                RoboRioSim.getVInVoltage(),
                0.02);
    }

    public void setIntakePreset(AlgaeIntakePresets preset) {
        this.currentPreset = preset;
        this.intakeMotorConfig = intakeMotorConfig.smartCurrentLimit(currentPreset.currentLimitA);
        this.intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public AlgaeIntakePresets getIntakePresets() {
        return currentPreset;
    }

    public boolean isHolding() {
        return holding_internal;
    }

    public BooleanSupplier getHoldingSupplier() {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isHolding();
            }
        };
    }

    public BooleanSupplier getNotHoldingSupplier() {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !isHolding();
            }
        };
    }

    public double getSensorDistance() {
        return intakeSensor.getMeasurement().distance_mm / 1000.0;
    }
}
