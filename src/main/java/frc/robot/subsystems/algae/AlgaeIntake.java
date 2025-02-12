package frc.robot.subsystems.algae;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Algae;
import frc.robot.Robot;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(Algae.Intake.MOTOR_PORT, MotorType.kBrushless);
    // sim motor
    private final SparkMaxSim simIntakeMotor = new SparkMaxSim(intakeMotor, Algae.Intake.PhysicalConstants.MOTOR);

    // physics simulation
    private final FlywheelSim simIntakePhysics = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    Algae.Intake.PhysicalConstants.MOTOR,
                    Algae.Intake.PhysicalConstants.MOI,
                    Algae.Intake.PhysicalConstants.GEARING),
            Algae.Intake.PhysicalConstants.MOTOR);

    private final DigitalInput intakeBeambreak = new DigitalInput(Algae.Intake.BEAMBREAK_PORT);

    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(
            Algae.Intake.POSITIVE_RATE_LIMIT,
            Algae.Intake.NEGATIVE_RATE_LIMIT,
            0);

    private double outputPercentage = 0.0;

    @Override
    public void periodic() {
        double output;
        // if no algae
        if (intakeBeambreak.get()) {
            output = intakeProfile.calculate(outputPercentage);
        } else {
            // hold
            output = Math.min(0.05, outputPercentage);
        }

        intakeMotor.set(output);
        if (Robot.isSimulation())
            simIntakeMotor.setAppliedOutput(output);
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

    public void setOutputPercentage(double outputPercentage) {
        this.outputPercentage = MathUtil.clamp(outputPercentage, -1, 1);
    }

    public double getOutputPercentage() {
        return outputPercentage;
    }

    public boolean isHolding() {
        return intakeBeambreak.get();
    }
}
