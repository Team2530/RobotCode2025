package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveStyle;
import frc.robot.util.Reef;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    private enum DriveState {
        Free,
        Locked,
    };

    private DriveState state = DriveState.Free;

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;

        DriveConstants.ROTATION_ASSIST.enableContinuousInput(-Math.PI / 2, Math.PI / 2);

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
        Translation2d xyRaw = new Translation2d(xbox.getLeftX(), xbox.getLeftY());
        Translation2d xySpeed = DeadBand(xyRaw, 0.15 / 2.f);
        double zSpeed = DeadBand(xbox.getRightX(), 0.1);
        double xSpeed = xySpeed.getX(); // xbox.getLeftX();
        double ySpeed = xySpeed.getY(); // xbox.getLeftY();

        // System.out.println("DRIVE!!");

        // double mag_xy = Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed);

        // xSpeed = mag_xy > 0.15 ? xSpeed : 0.0;
        // ySpeed = mag_xy > 0.15 ? ySpeed : 0.0;
        // zSpeed = Math.abs(zSpeed) > 0.15 ? zSpeed : 0.0;

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        // double dmult = dsratelimiter.calculate(xbox.getRightBumper() ? 1.0 :
        // SLOWMODE_MULT);
        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        if (xbox.getXButton()) {
            swerveSubsystem.zeroHeading();
            Translation2d pospose = swerveSubsystem.getPose().getTranslation();
            swerveSubsystem.odometry.resetPosition(swerveSubsystem.getRotation2d(),
                    swerveSubsystem.getModulePositions(),
                    new Pose2d(pospose, new Rotation2d(FieldConstants.getAlliance() == Alliance.Blue ? 0.0 : Math.PI)));
            // swerveSubsystem.resetOdometry(new Pose2d(1.38, 5.55, new Rotation2d()));
            // swerveSubsystem.zeroHeading();
        }

        ChassisSpeeds speeds;

        switch (swerveSubsystem.getDriveStyle()) {
            case FIELD_ORIENTED:
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, zSpeed,
                        new Rotation2d(
                                -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET)
                                        .getRadians()));
                break;
            case ROBOT_ORIENTED:
                speeds = new ChassisSpeeds(-xSpeed, ySpeed, -zSpeed);
                break;
            case ROTATION_ASSIST:
                // ? X/Y have been left in here as comments in case you find that you want them!
                Pose2d closestPose = swerveSubsystem.getPose().nearest(new ArrayList<Pose2d>(Reef.branches.values()));

                double zError = swerveSubsystem.getPose().minus(closestPose).getRotation().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET).getRadians();
                // double xError = swerveSubsystem.getPose().getX() -
                // swerveSubsystem.getTargetPose().getX();
                // double yError = swerveSubsystem.getPose().getY() -
                // swerveSubsystem.getTargetPose().getY();

                // double yAssist = DriveConstants.TRANSLATION_ASSIST.calculate(-xError) *
                // xbox.getLeftTriggerAxis();
                // double xAssist = DriveConstants.TRANSLATION_ASSIST.calculate(yError) *
                // xbox.getLeftTriggerAxis();
                double zAssist = DriveConstants.ROTATION_ASSIST.calculate(zError);// * xbox.getLeftTriggerAxis();
                // speeds = new ChassisSpeeds(-xSpeed + xAssist, ySpeed + yAssist, -zSpeed +
                // zAssist);

                // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed + yAssist, xSpeed +
                // xAssist,
                // zSpeed + zAssist,
                // new Rotation2d(
                // -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET)
                // .getRadians()));

                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed,
                        zSpeed - zAssist,
                        new Rotation2d(
                                -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET)
                                        .getRadians()));

                break;

            default:
                speeds = null;
                break;
        }

        // State transition logic
        switch (state) {
            case Free:
                // state = xbox.getRightBumper() ? DriveState.Locked : DriveState.Free;
                break;
            case Locked:
                state = ((xyRaw.getNorm() > 0.15) && !xbox.getBButton()) ? DriveState.Free : DriveState.Locked;
                break;
        }

        // Drive execution logic
        switch (state) {
            case Free:
                SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
                swerveSubsystem.setModules(calculatedModuleStates);
                break;
            case Locked:
                swerveSubsystem.setXstance();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
}
