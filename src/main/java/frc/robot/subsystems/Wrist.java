// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.InaccessibleObjectException;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.PIDSubsystem;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  private final Phi phi;
  private final Theta theta;
  private final TalonFX intake;

  private double intakeSpeed;
  private boolean enabled = true;

  public enum IntakeMode {
    INTAKE(0.2),
    STOP(0.0),
    HOLD(0.1),
    REVERSE(-0.2);

    private double speed;

    private IntakeMode(double speed) {
      this.speed = speed;
    }
  }

  /**
   * Contains Phi, Theta angles for reef scoring
   * TODO: Need to be determined with the actual robot, these are just guesses
   */
  public enum Presets {
    L1(90, -85),
    L2(90, -85),
    L3(90, -85),
    L4(90, -90);

    private double phi, theta;

    private Presets(double phi, double theta) {
      this.phi = phi;
      this.theta = theta;
    }
  }

  public Wrist() {
    this.phi = new Phi();
    this.theta = new Theta();
    this.intake = new TalonFX(WristConstants.INTAKE_ID);
  }

  @Override
  public void periodic() {
    if (enabled) {
      intake.set(intakeSpeed);
    }
  }

  public void disable() {
    phi.disable();
    theta.disable();
    stopIntake();
    enabled = false;
  }

  public void setIntakeMode(IntakeMode mode) {
    intakeSpeed = mode.speed;
  }

  public void enable() {
    phi.enable();
    theta.enable();
    enabled = true;
  }

  public void setIntakeSpeed(double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
  }

  public void stopIntake() {
    intakeSpeed = 0;
  }

  /**
   * Sets {@link}Presets which is a easier way to set the value of the arm
   * 
   * @param preset the L1-L4 scoring preset
   */
  public void setPreset(Presets preset) {
    setGoal(preset.phi, preset.theta);
  }

  /**
   * Sets the goal for both intake DOF's (refrenced from when arm is straight up)
   * 
   * @param phi   XY plane rotation for the wrist
   * @param theta YZ Plane rotation for the wrist
   */
  public void setGoal(double phi, double theta) {
    this.phi.setGoal(phi);
    this.theta.setGoal(theta);
  }

  public void setPhi(double phi) {
    this.phi.setGoal(phi);

  }

  public void setTheta(double theta) {
    this.theta.setGoal(theta);
  }

  public boolean hasCoral() {
    return intake.getTorqueCurrent().getValueAsDouble() > 4;
  }

  public double getPhiPosition() {
    return phi.getPosition();
  }

  public double getThetaPosition() {
    return theta.getPosition();
  }

  // ---------------------- Inner Wrist PID Classes ---------------------- \\

  private class Phi extends PIDSubsystem {
    private final SparkFlex phiMotor;
    private final AbsoluteEncoder encoder;

    public Phi() {
      super(WristConstants.PHI_CONTROLLER);
      phiMotor = new SparkFlex(WristConstants.PHI_ID, MotorType.kBrushless);
      // Config intake motor
      SparkMaxConfig phiConfig = new SparkMaxConfig();
      phiConfig.idleMode(IdleMode.kCoast).inverted(false);
      phiMotor.configure(phiConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      encoder = phiMotor.getAbsoluteEncoder();
    }

    @Override
    public void periodic() {
      super.periodic();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
      phiMotor.set(output / 12d);
    }

    @Override
    protected double getMeasurement() {
      return Units.rotationsToRadians(encoder.getPosition());
    }

    public double getPosition() {
      return getMeasurement();
    }

  }

  private class Theta extends PIDSubsystem {
    private final SparkFlex thetaMotor;
    private final RelativeEncoder encoder;

    public Theta() {
      super(WristConstants.THETA_CONTROLLER);
      thetaMotor = new SparkFlex(WristConstants.THETA_ID, MotorType.kBrushless);

      SparkMaxConfig thetaConfig = new SparkMaxConfig();
      thetaConfig.idleMode(IdleMode.kBrake).inverted(false);
      thetaMotor.configure(thetaConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      encoder = thetaMotor.getEncoder();
    }

    @Override
    public void periodic() {
      super.periodic();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
      thetaMotor.set(output / 12d);
    }

    @Override
    protected double getMeasurement() {
      return Units.rotationsToRadians(encoder.getPosition());
    }

    public double getPosition() {
      return getMeasurement();
    }
  }
}