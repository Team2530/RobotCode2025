// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.PIDSubsystem;

public class Arm extends PIDSubsystem {
  private final SparkFlex armMotor;
  private final RelativeEncoder encoder;

  /** Creates a new Arm. */
  public Arm() {
    super(ArmConstants.ARM_CONTROLLER);

    armMotor = new SparkFlex(WristConstants.PHI_ID, MotorType.kBrushless);
      // Config intake motor
      SparkMaxConfig armConfig = new SparkMaxConfig();
      armConfig.idleMode(IdleMode.kCoast).inverted(false);
      armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      encoder = armMotor.getEncoder();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public void setDegrees(double degrees) {
    setGoal(degrees);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    double feed = ArmConstants.ARM_FEEDFORWARD.calculate(setpoint.position + 90, setpoint.velocity);
    armMotor.set((output + feed) / 12d);
  }

  @Override
  protected double getMeasurement() {
    return Units.rotationsToRadians(encoder.getPosition());
  }

  public double getPosition() {
    return getMeasurement();
  }
}
