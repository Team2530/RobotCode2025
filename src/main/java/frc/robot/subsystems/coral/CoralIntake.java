package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
            Coral.Intake.PhysicalConstants.GEARING
        ),
        Coral.Intake.PhysicalConstants.MOTOR
    );
 
    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(
        Coral.Intake.POSITIVE_RATE_LIMIT,
        Coral.Intake.NEGATIVE_RATE_LIMIT,
        0
    );

    public CoralIntake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private double outputPercentage = 0.0;

    @Override
    public void periodic() {
        // if no coral 
        if (this.isHolding()) {
            double output = intakeProfile.calculate(outputPercentage);
            intakeMotor.set(output);
        } else {
            // hold
            intakeMotor.set(Math.min(0.05, outputPercentage));
        }

    }

    @Override
    public void simulationPeriodic() {
        simIntakeMotor = intakeMotor.getSimState();
        simIntakeMotor.setSupplyVoltage(RoboRioSim.getVInVoltage());
        
        // update physics
        simIntakePhysics.setInput(simIntakeMotor.getMotorVoltage() * RoboRioSim.getVInVoltage());
        simIntakePhysics.update(0.02);

        // update sim objects
        //                              rpm to rps
        simIntakeMotor.setRotorVelocity(simIntakePhysics.getAngularVelocityRPM() / 60);
    }

    public void setOutputPercentage(double outputPercentage) {
        this.outputPercentage = MathUtil.clamp(outputPercentage, -1, 1);
    }

    public double getOutputPercentage() {
        return outputPercentage;
    }

    public boolean isHolding() {
        return intakeMotor.getReverseLimit().getValue().value == 0;
    }
}
