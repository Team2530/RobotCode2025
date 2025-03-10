package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(Climber.MOTOR_PORT, MotorType.kBrushless);
    private SparkMaxConfig climberConfig = new SparkMaxConfig();
    private RelativeEncoder climberEncoder;
    XboxController operator;

    boolean isTested = false;

//     public ClimberSubsystem(XboxController operator) {
//         this.operator = operator;
//         climberMotor.configure(climberConfig.idleMode(IdleMode.kBrake)
//                 .apply(new SoftLimitConfig().reverseSoftLimit(Constants.Climber.DEPLOY_SOFT_LIMIT)
//                         .reverseSoftLimitEnabled(true).forwardSoftLimit(0.0)
//                         .forwardSoftLimitEnabled(true))
//                 .apply(
//                         new EncoderConfig().positionConversionFactor(1.0 / Constants.Climber.GEAR_RATIO)
//                                 .velocityConversionFactor(
//                                         1.0 / Constants.Climber.GEAR_RATIO)),
//                 ResetMode.kResetSafeParameters,
//                 PersistMode.kPersistParameters);

                public ClimberSubsystem(XboxController operator) {


                this.operator = operator;


                climberMotor.configure(climberConfig.idleMode(IdleMode.kBrake)


                                .apply(new SoftLimitConfig().reverseSoftLimit(Constants.Climber.DEPLOY_SOFT_LIMIT)


                                                .reverseSoftLimitEnabled(true).forwardSoftLimit(0.0)


                                                .forwardSoftLimitEnabled(true))


                                .apply(


                                                new EncoderConfig()


                                                                .positionConversionFactor(


                                                                                1.0 / Constants.Climber.GEAR_RATIO)


                                                                .velocityConversionFactor(


                                                                                1.0 / Constants.Climber.GEAR_RATIO)),


                                ResetMode.kResetSafeParameters,


                                PersistMode.kPersistParameters);

        climberEncoder = climberMotor.getEncoder();
    }

    private double output = 0.0;
    private boolean deployed = false;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", climberEncoder.getPosition());
        if (climberEncoder.getPosition() < Constants.Climber.CLIMB_SOFT_LIMIT &&
                !deployed) {
            deployed = true;
            climberMotor.configure(climberConfig.apply(climberConfig.softLimit.forwardSoftLimit(
                    Constants.Climber.CLIMB_SOFT_LIMIT)),
                    ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        if (DriverStation.isTestEnabled() & !isTested) {


                        climberMotor.configure(


                                        climberConfig.apply(


                                                        climberConfig.softLimit.forwardSoftLimitEnabled(false)


                                                                        .reverseSoftLimitEnabled(false)),


                                        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


                        isTested = false;


                }

        if (!Constants.Climber.DBG_DISABLED)


                        climberMotor.set(operator.getLeftBumper()


                                        ? (MathUtil.applyDeadband(


                                                        operator.getLeftY(), 0.05)


                                                        * (operator.getLeftY() < 0.0 ? 0.7 : 1.0))


                                        : 0.0);


        }

    public void resetClimberDeploy() {
        climberMotor.configure(climberConfig.apply(climberConfig.softLimit.forwardSoftLimit(0.0)),
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setOutput(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }
}
