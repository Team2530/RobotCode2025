package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
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
import frc.robot.Constants.Coral;

public class CoralArm extends SubsystemBase {
        private final SparkFlex pivotMotor = new SparkFlex(Coral.Pivot.MOTOR_PORT, MotorType.kBrushless);
        private final SparkMax rollMotor = new SparkMax(Coral.Roll.MOTOR_PORT, MotorType.kBrushless);
        private final SparkMax pitchMotor = new SparkMax(Coral.Pitch.MOTOR_PORT, MotorType.kBrushless);
        // sim motors
        private final SparkFlexSim simPivotMotor = new SparkFlexSim(pivotMotor, Coral.Pivot.PhysicalConstants.MOTOR);
        private final SparkMaxSim simRollMotor = new SparkMaxSim(rollMotor, Coral.Roll.PhysicalConstants.MOTOR);
        private final SparkMaxSim simPitchMotor = new SparkMaxSim(pitchMotor, Coral.Pitch.PhysicalConstants.MOTOR);

        private final CANcoder pivotEncoder = new CANcoder(Coral.Pivot.ENCODER_PORT);
        private final SparkAnalogSensor rollEncoder = rollMotor.getAnalog();
        private final SparkAnalogSensor pitchEncoder = pitchMotor.getAnalog();
        // sim encoders
        private final SparkAbsoluteEncoderSim simRollEncoder = new SparkAbsoluteEncoderSim(rollMotor);
        private final SparkAbsoluteEncoderSim simPitchEncoder = new SparkAbsoluteEncoderSim(pitchMotor);

        // physics simulations
        private final SingleJointedArmSim simPivotPhysics = new SingleJointedArmSim(
                        Coral.Pivot.PhysicalConstants.MOTOR,
                        Coral.Pivot.PhysicalConstants.NET_REDUCTION,
                        Coral.Pivot.PhysicalConstants.MOI,
                        Coral.Pivot.PhysicalConstants.ARM_LENGTH_METERS,
                        Units.degreesToRadians(Coral.Pivot.MAXIMUM_ANGLE * -1),
                        Units.degreesToRadians(Coral.Pivot.MAXIMUM_ANGLE),
                        true,
                        0);
        private final SingleJointedArmSim simRollPhysics = new SingleJointedArmSim(
                        Coral.Roll.PhysicalConstants.MOTOR,
                        Coral.Roll.PhysicalConstants.NET_REDUCTION,
                        Coral.Roll.PhysicalConstants.MOI,
                        Coral.Roll.PhysicalConstants.ARM_LENGTH_METERS,
                        Units.degreesToRadians(Coral.Roll.MAXIMUM_ANGLE * -1),
                        Units.degreesToRadians(Coral.Roll.MAXIMUM_ANGLE),
                        false,
                        0);
        private final SingleJointedArmSim simPitchPhysics = new SingleJointedArmSim(
                        Coral.Pitch.PhysicalConstants.MOTOR,
                        Coral.Pitch.PhysicalConstants.NET_REDUCTION,
                        Coral.Pitch.PhysicalConstants.MOI,
                        Coral.Pitch.PhysicalConstants.ARM_LENGTH_METERS,
                        Units.degreesToRadians(Coral.Pitch.MAXIMUM_ANGLE * -1),
                        Units.degreesToRadians(Coral.Pitch.MAXIMUM_ANGLE),
                        false,
                        0);

        private final SparkFlexConfig pivotConfig = new SparkFlexConfig();
        private final SparkMaxConfig rollConfig = new SparkMaxConfig();
        private final SparkMaxConfig pitchConfig = new SparkMaxConfig();

        private final ProfiledPIDController pivotPID = Coral.Pivot.PID;
        private final ArmFeedforward pivotFeedforward = Coral.Pivot.FEEDFORWARD;
        private final ProfiledPIDController rollPID = Coral.Roll.PID;
        private final ProfiledPIDController pitchPID = Coral.Pitch.PID;

        private double pivotGoal = 0.0; // Rads
        private double rollGoal = 0.0; // Rads
        private double pitchGoal = 0.0; // Rads

        public CoralArm() {
                AnalogSensorConfig wristEncConfig = new AnalogSensorConfig()
                                .positionConversionFactor((Math.PI * 2) / 5.0)
                                .velocityConversionFactor((Math.PI * 2) / 5.0);

                pivotMotor.configure(
                                pivotConfig.idleMode(IdleMode.kBrake).apply(
                                                new EncoderConfig()
                                                                .positionConversionFactor(
                                                                                (1.0 * 2.0 * Math.PI)
                                                                                                / Constants.Coral.Pivot.PhysicalConstants.NET_REDUCTION)
                                                                .velocityConversionFactor(
                                                                                (60.0 * 2.0 * Math.PI)
                                                                                                / Constants.Coral.Pivot.PhysicalConstants.NET_REDUCTION)), // Encoder
                                                                                                                                                           // ->
                                // Rotations &
                                // Seconds
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                rollMotor.configure(
                                rollConfig.idleMode(IdleMode.kBrake)
                                                .apply(wristEncConfig.inverted(Constants.Coral.Roll.ENCODER_INVERTED)),
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                pitchMotor.configure(pitchConfig
                                .idleMode(IdleMode.kBrake).apply(wristEncConfig
                                                .inverted(Constants.Coral.Pitch.ENCODER_INVERTED)),
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                /*
                 * .apply(new EncoderConfig()
                 * .positionConversionFactor(
                 * (1.0 * 2.0 * Math.PI)
                 * / Constants.Coral.Roll.PhysicalConstants.NET_REDUCTION)
                 * .velocityConversionFactor((60.0 * 2.0 * Math.PI)
                 * / Constants.Coral.Roll.PhysicalConstants.NET_REDUCTION)),
                 */
                // Not sure how to get

                // Disabling this for now because we don't want the wrist wire bundle to explode
                // rollPID.enableContinuousInput(-180, 180);

                // TODO: Are these reasonable? who knows.
                pivotPID.setTolerance(Units.degreesToRadians(0.5));
                rollPID.setTolerance(Units.degreesToRadians(0.5));
                pitchPID.setTolerance(Units.degreesToRadians(0.5));
        }

        @Override
        public void periodic() {
                // if entering the frame border
                // && wrist is at neutral
                if ((pivotPID.getGoal().position != pivotGoal)
                                && (rollPID.atGoal() && pitchPID.atGoal())) {
                        // allow arm to enter frame border
                        pivotPID.setGoal(pivotGoal);
                } else if ( // if exiting the frame border && has exited the frame border
                ((rollGoal != rollPID.getGoal().position) || (pitchGoal != pitchPID.getGoal().position))
                                && (Math.abs(this.getPitchPositionDegrees()) > Coral.Pivot.FRAME_BORDER_ANGLE)) {
                        // set roll and pitch back to their goals
                        rollPID.setGoal(rollGoal);
                        pitchPID.setGoal(pitchGoal);
                }

                double pivotPosition = Units
                                .rotationsToRadians((pivotEncoder.getAbsolutePosition().getValueAsDouble()
                                                * (Constants.Coral.Pivot.ENCODER_INVERTED ? -1.0 : 1.0)));
                double pivotFFout = pivotFeedforward.calculate(pivotPID.getSetpoint().position,
                                pivotPID.getSetpoint().velocity);
                double pivotPIDout = pivotPID.calculate(pivotPosition);

                SmartDashboard.putNumber("Coral/Pivot/pid_out", pivotPIDout);
                SmartDashboard.putNumber("Coral/Pivot/ff_out", pivotFFout);
                SmartDashboard.putNumber("Coral/Pivot/out", pivotFFout + pivotPIDout);
                SmartDashboard.putNumber("Coral/Pivot/position", pivotPosition);
                SmartDashboard.putNumber("Coral/Pivot/target", pivotPID.getSetpoint().position);
                SmartDashboard.putNumber("Coral/Pivot/goal", pivotPID.getGoal().position);

                // run the motors
                if (!Constants.Coral.Pivot.DBG_DISABLED)
                        pivotMotor.setVoltage(pivotPIDout + pivotFFout);

                double rollPIDout = rollPID.calculate(readRollEncoderPosition());
                SmartDashboard.putNumber("Coral/Roll/position", readRollEncoderPosition());
                SmartDashboard.putNumber("Coral/Roll/target", rollPID.getSetpoint().position);
                SmartDashboard.putNumber("Coral/Roll/goal", rollPID.getGoal().position);
                SmartDashboard.putNumber("Coral/Roll/out", rollPIDout);
                if (!Constants.Coral.Roll.DBG_DISABLED)
                        rollMotor.setVoltage(rollPIDout);

                double pitchPIDout = pitchPID.calculate(readPitchEncoderPosition());
                SmartDashboard.putNumber("Coral/Pitch/position", readPitchEncoderPosition());
                SmartDashboard.putNumber("Coral/Pitch/target", pitchPID.getSetpoint().position);
                SmartDashboard.putNumber("Coral/Pitch/goal", pitchPID.getGoal().position);
                SmartDashboard.putNumber("Coral/Pitch/out", pitchPIDout);
                if (!Constants.Coral.Pitch.DBG_DISABLED)
                        pitchMotor.setVoltage(pitchPIDout);
        }

        @Override
        public void simulationPeriodic() {
                // update physics
                simPivotPhysics.setInput(simPivotMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
                simRollPhysics.setInput(simRollMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
                simPitchPhysics.setInput(simPitchMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());

                simPivotPhysics.update(0.02);
                simRollPhysics.update(0.02);
                simPitchPhysics.update(0.02);

                // update sim objects
                simPivotMotor.iterate(
                                simPivotPhysics.getVelocityRadPerSec(),
                                RoboRioSim.getVInVoltage(),
                                0.02);
                simRollMotor.iterate(
                                simRollPhysics.getVelocityRadPerSec(),
                                RoboRioSim.getVInVoltage(),
                                0.02);
                simPitchMotor.iterate(
                                simPitchPhysics.getVelocityRadPerSec(),
                                RoboRioSim.getVInVoltage(),
                                0.02);
        }

        // this should be relative to straight upwards.
        // i.e. 0 should be straight vertical,
        // 20 would be to the right, -20 to the left
        public void setPivotGoalDegrees(double goal) {
                double goalRads = Units.degreesToRadians(goal);
                pivotGoal = MathUtil.clamp(goalRads,
                                Coral.Pivot.MAXIMUM_ANGLE, -1 * Coral.Pivot.MAXIMUM_ANGLE);
                // if passing through the frame border
                if (((this.getPivotPositionDegrees() < 0) == (pivotGoal < 0))
                                && (Math.abs(this.getPivotPositionDegrees()) < Coral.Pivot.FRAME_BORDER_ANGLE)) {
                        // set the pivot to stop at the frame border to allow the wrist to move
                        // neutral
                        pivotPID.setGoal(
                                        Coral.Pivot.FRAME_BORDER_ANGLE
                                                        * (pivotGoal < 0 ? -1 : 1));
                        rollPID.setGoal(0);
                        pitchPID.setGoal(0);
                } else {
                        pivotPID.setGoal(pivotGoal);
                }
        }

        // similarly, this ranges from -180 to 180
        public void setRollGoalDegrees(double goal) {
                double goalRads = Units.degreesToRadians(goal);
                rollGoal = MathUtil.clamp(goalRads, Coral.Roll.MAXIMUM_ANGLE * -1, Coral.Roll.MAXIMUM_ANGLE);
                // if outside of frame border
                if (Math.abs(this.getPivotPositionDegrees()) > Coral.Pivot.FRAME_BORDER_ANGLE) {
                        // start moving to goal
                        rollPID.setGoal(rollGoal);
                }

        }

        public void setPitchGoalDegrees(double goal) {
                double goalRads = Units.degreesToRadians(goal);
                pitchGoal = MathUtil.clamp(goalRads, Coral.Pitch.MAXIMUM_ANGLE * -1, Coral.Pitch.MAXIMUM_ANGLE);
                // if outside the frame border
                if (Math.abs(this.getPivotPositionDegrees()) > Coral.Pivot.FRAME_BORDER_ANGLE) {
                        // start moving to goal
                        pitchPID.setGoal(pitchGoal);
                }
        }

        public double getPivotGoalDegrees() {
                return Units.radiansToDegrees(pivotGoal);
        }

        public double getRollGoalDegrees() {
                return Units.radiansToDegrees(rollGoal);
        }

        public double getPitchGoalDegrees() {
                return Units.radiansToDegrees(pitchGoal);
        }

        public double getPivotPositionDegrees() {
                return Units.rotationsToDegrees(Robot.isReal()
                                ? (pivotEncoder.getAbsolutePosition().getValueAsDouble()
                                                * (Constants.Coral.Pivot.ENCODER_INVERTED ? -1.0 : 1.0))
                                : simPivotPhysics.getAngleRads());
        }

        public double getRollPositionDegrees() {
                return Units.radiansToDegrees(Robot.isReal()
                                ? readRollEncoderPosition()
                                : simRollEncoder.getPosition());
        }

        public double getPitchPositionDegrees() {
                return Units.radiansToDegrees(Robot.isReal()
                                ? readPitchEncoderPosition()
                                : simPitchEncoder.getPosition());
        }

        public boolean isPitchInPosition() {
                return pitchPID.atGoal();
        }

        public boolean isRollInPosition() {
                return rollPID.atGoal();
        }

        public boolean isPivotInPosition() {
                return pivotPID.atGoal();
        }

        public double readPitchEncoderPosition() {
                return pitchEncoder.getPosition() + Constants.Coral.Pitch.ENCODER_OFFSET_RADS;
        }

        public double readRollEncoderPosition() {
                return rollEncoder.getPosition() + Constants.Coral.Roll.ENCODER_OFFSET_RADS;
        }
}
