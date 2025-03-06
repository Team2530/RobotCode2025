package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Reef;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    public static enum DriveStyle {
        FIELD_ORIENTED,
        ROTATION_ASSIST,
    };

    private DriveStyle driveStyle = DriveStyle.FIELD_ORIENTED;
    private PIDController rotationAssist = new PIDController(
            DriveConstants.ROTATION_ASSIST.kP,
            DriveConstants.ROTATION_ASSIST.kI,
            DriveConstants.ROTATION_ASSIST.kD);
    private PIDController translationAssist = new PIDController(
            DriveConstants.TRANSLATION_ASSIST.kP,
            DriveConstants.TRANSLATION_ASSIST.kI,
            DriveConstants.TRANSLATION_ASSIST.kD);

    private boolean isXstance = false;

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;

        rotationAssist.enableContinuousInput(-Math.PI, Math.PI);
        dsratelimiter.reset(SLOWMODE_MULT);

        addRequirements(swerveSubsystem);
    }

    double clamp(double v, double mi, double ma) {
        return (v < mi) ? mi : (v > ma ? ma : v);
    }

    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            // TODO: Check is it sqrt2 or 1.0...
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        Translation2d xyRaw = new Translation2d(-xbox.getLeftY(), -xbox.getLeftX());
        double zSpeed = -MathUtil.applyDeadband(xbox.getRightX(), 0.1);
        double xSpeed = MathUtil.applyDeadband(xyRaw.getX(), 0.08); // xbox.getLeftX();
        double ySpeed = MathUtil.applyDeadband(xyRaw.getY(), 0.08); // xbox.getLeftY();

        SmartDashboard.putString("Drive XY", xyRaw.toString());
        SmartDashboard.putNumber("Drive Z", zSpeed);

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;
        SmartDashboard.putNumber("Drive Multiplier", dmult);

        if (xbox.getXButton()) {
            swerveSubsystem.zeroHeading();
            Translation2d pospose = swerveSubsystem.getOdometryPose().getTranslation();
            swerveSubsystem.odometry.resetPosition(swerveSubsystem.getGyroRotation2d(),
                    swerveSubsystem.getModulePositions(),
                    new Pose2d(pospose, new Rotation2d(FieldConstants.getAlliance() == Alliance.Blue ? 0.0 : Math.PI)));
        }
        if (xbox.getStartButton()) {
            SmartDashboard.putBoolean("Ham called", true);
        }

        ChassisSpeeds speeds = new ChassisSpeeds();
        switch (driveStyle) {
            case FIELD_ORIENTED:
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, zSpeed,
                        swerveSubsystem.getGyroRotation2d());
                break;
            case ROTATION_ASSIST:
                Translation2d reefCenter = AllianceFlipUtil.apply(Reef.center);
                double reefAngleRot = swerveSubsystem.getOdometryPose().getTranslation().minus(
                        reefCenter).getAngle().getRotations();

                double targetAngle = MathUtil
                        .angleModulus(Units.rotationsToRadians(Math.floor((reefAngleRot + 1.0 / 12.0) * 6.0) / 6.0))
                        + Math.PI / 2.0;

                SmartDashboard.putNumber("AutoRotate Target Angle", Units.radiansToDegrees(targetAngle));

                SwerveModuleState rotationState = new SwerveModuleState(2.0, new Rotation2d(targetAngle));
                rotationState.optimize(swerveSubsystem.getOdometryPose().getRotation());

                double zAssist = rotationAssist.calculate(swerveSubsystem.getOdometryPose().getRotation().getRadians(),
                        rotationState.angle.getRadians());
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, zSpeed + zAssist,
                        swerveSubsystem.getGyroRotation2d());
                break;

            default:
                break;
        }

        // State transition logic
        isXstance = false;
        if (isXstance)
            isXstance = !((xyRaw.getNorm() > 0.08) && !xbox.getBButton());

        // Drive execution logic

        if (isXstance) {
            swerveSubsystem.setXstance();
        } else {
            swerveSubsystem.setChassisSpeeds(speeds);
        }
    }

    public void setDriveStyle(DriveStyle style) {
        this.driveStyle = style;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
}
