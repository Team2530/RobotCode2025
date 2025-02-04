package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class SwerveModule {
    private final TalonFX driveMotor;
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
    private final TalonFXConfigurator driveConfigurator;
    private final MotorOutputConfigs driveConfig;
    private final SparkMax steerMotor;
    private final SparkMaxConfig steerConfig;

    // private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private double driveEncSim = 0;
    private double steerEncSim = 0;

    private final com.ctre.phoenix6.hardware.CANcoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;
    private final boolean motor_inv;

    private final PIDController steerPID;

    private static int moduleNumber = 0;
    int thisModuleNumber;

    SlewRateLimiter turnratelimiter = new SlewRateLimiter(4.d);

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed, boolean motorReversed) {
        // driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        driveMotor = new TalonFX(driveCanID);
        driveConfigurator = driveMotor.getConfigurator();
        driveConfig = new MotorOutputConfigs();
        driveConfig.Inverted = motorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.NeutralMode = NeutralModeValue.Brake;
        driveConfigurator.apply(driveConfig);

        // driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor = new SparkMax(steerCanID, MotorType.kBrushless);
        steerConfig = new SparkMaxConfig();
        steerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        steerConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.STEER_ROTATION_TO_RADIANS)
                .velocityConversionFactor(SwerveModuleConstants.STEER_RADIANS_PER_MINUTE);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.motor_inv = motorReversed;
        // driveMotorEncoder = driveMotor.get();
        steerMotorEncoder = steerMotor.getEncoder();

        // Reset encoder offsets possibly set in Tuner X
        absoluteEncoder = new com.ctre.phoenix6.hardware.CANcoder(absoluteEncoderPort);
        com.ctre.phoenix6.configs.CANcoderConfiguration cfg = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        cfg.MagnetSensor = new com.ctre.phoenix6.configs.MagnetSensorConfigs();
        cfg.MagnetSensor.MagnetOffset = 0.0f;
        absoluteEncoder.getConfigurator().apply(cfg);
        // CANcoderConfigurator configurator = absoluteEncoder.getConfigurator();

        this.motorOffsetRadians = motorOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        // driveMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER);
        // driveMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_MINUTE);
        // TalonFXConfiguration talon_cfg = new TalonFXConfiguration();
        // driveMotor.getConfigurator().apply(cfg);

        steerPID = new PIDController(SwerveModuleConstants.MODULE_KP, 0, SwerveModuleConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        thisModuleNumber = moduleNumber;
        moduleNumber++;

        resetEncoders();
    }

    public void simulate_step() {
        driveEncSim += 0.02 * driveMotor.get() * (DriveConstants.MAX_MODULE_VELOCITY);
        steerEncSim += 0.02 * steerMotor.get() * (10.0);
    }

    public double getDrivePosition() {
        if (Robot.isSimulation())
            return driveEncSim;
        // return driveMotorEncoder.getPosition();
        // TODO: Do the conversion in the motor
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelocity() {
        // return driveMotorEncoder.getVelocity();
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getSteerPosition() {
        if (Robot.isSimulation())
            return steerEncSim;
        return steerMotorEncoder.getPosition();
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble());// * (Math.PI /
        // 180.d);
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        // driveMotorEncoder.setPosition(0);
        driveMotor.setPosition(0.0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(),
                new Rotation2d(-getSteerPosition()).rotateBy(DriveConstants.NAVX_ANGLE_OFFSET.times(-1)));
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        state.optimize(new Rotation2d(getSteerPosition()));
        double drive_command = state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY;
        // SmartDashboard.putNumber("Module " + Integer.toString(this.thisModuleNumber) + " Drive", drive_command);
        driveMotor.set(drive_command * (motor_inv ? -1.0 : 1.0));

        // This is stupid
        // steerPID.setP(Constants.SwerveModuleConstants.MODULE_KP *
        // Math.abs(drive_command));
        double steercmd = steerPID.calculate(getSteerPosition(), state.angle.getRadians());
        if (Robot.isSimulation()) {
            steerMotor.set(steercmd);
        } else {
            steerMotor.setVoltage(12 * steercmd);
        }
        // SmartDashboard.putNumber("Abs" + thisModuleNumber,
        // getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Drive" + thisModuleNumber, drive_command);
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
