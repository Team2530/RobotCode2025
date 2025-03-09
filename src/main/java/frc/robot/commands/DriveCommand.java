package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.CoralStation;
import frc.robot.util.Reef;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    public static enum DriveStyle {
        FIELD_ORIENTED,
        REEF_ASSIST,
        INTAKE_ASSIST,
        CORAL_SPOT_ASSIST
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

    final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);
    HolonomicDriveController a = new HolonomicDriveController(null, null, null);
    HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(6.28, 3.14)));
    

    private boolean isXstance = false;

    private int selectedTag = PoseConstants.defaultSelectedTag;

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

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

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
            case REEF_ASSIST:
                Translation2d reefCenter = AllianceFlipUtil.apply(Reef.center);
                double reefAngleRot = swerveSubsystem.getOdometryPose().getTranslation().minus(
                        reefCenter).getAngle().getRotations();

                double targetAngle = MathUtil
                        .angleModulus(Units.rotationsToRadians(Math.floor((reefAngleRot + 1.0 / 12.0) * 6.0) / 6.0))
                        - Math.PI / 2.0;

                SmartDashboard.putNumber("AutoRotate Target Angle", Units.radiansToDegrees(targetAngle));

                SwerveModuleState rotationState = new SwerveModuleState(1.5, new Rotation2d(targetAngle));
                // NOTE: Remove this to dedicate a scoring side
                // rotationState.optimize(swerveSubsystem.getOdometryPose().getRotation());

                double zAssist = MathUtil
                        .clamp(rotationAssist.calculate(swerveSubsystem.getOdometryPose().getRotation().getRadians(),
                                rotationState.angle.getRadians()), -0.5 * DriveConstants.MAX_ROBOT_RAD_VELOCITY,
                                0.5
                                        * DriveConstants.MAX_ROBOT_RAD_VELOCITY);
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, zSpeed + zAssist,
                        swerveSubsystem.getGyroRotation2d());
                break;
            case INTAKE_ASSIST:
                // double zTarget =
                // if (AllianceFlipUtil.shouldFlip())
                // zTarget += Math.PI;
                Pose2d csRight = AllianceFlipUtil.apply(CoralStation.coralStations[0]);
                Pose2d csLeft = AllianceFlipUtil.apply(CoralStation.coralStations[1]);
                // CoralStation.putToShuffleboard();

                Pose2d csTarget = (swerveSubsystem.getOdometryPose().getTranslation()
                        .getDistance(csRight.getTranslation()) < swerveSubsystem.getOdometryPose().getTranslation()
                                .getDistance(csLeft.getTranslation())) ? csRight : csLeft;

                // TODO: Optimize if need be
                double targetRotation = csTarget.getRotation().rotateBy(Rotation2d.fromDegrees(90.0)).getRadians();

                double zPid = MathUtil
                        .clamp(rotationAssist.calculate(swerveSubsystem.getOdometryPose().getRotation().getRadians(),
                                targetRotation), -0.5 * DriveConstants.MAX_ROBOT_RAD_VELOCITY,
                                0.5
                                        * DriveConstants.MAX_ROBOT_RAD_VELOCITY);
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, zSpeed + zPid,
                        swerveSubsystem.getGyroRotation2d());
            case CORAL_SPOT_ASSIST:
                // TODO: make a tag switcher somehow
                // TODO: offload the goal position into constants or something, shouldn't run periodically as it's a 1 time calculation. CBA to do now.
                Pose2d thePose = Constants.PoseConstants.blueCoralScores[0]; // i.e., A 
                Pose2d currPose = swerveSubsystem.odometry.getEstimatedPosition();
                edu.wpi.first.math.trajectory.Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    currPose, 
                    List.of(),  // No intermediate waypoints (just a straight line)
                    thePose, 
                    config
                );
                State goal = trajectory.sample(Timer.getFPGATimestamp()-Constants.PoseConstants.startTime);
                ChassisSpeeds adjustedSpeeds = controller.calculate(
                    currPose, goal, currPose.getRotation()); // what is the last parameter? TODO: fix
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed+adjustedSpeeds.vxMetersPerSecond, ySpeed+adjustedSpeeds.vyMetersPerSecond, zSpeed + adjustedSpeeds.omegaRadiansPerSecond,
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

    public Pose2d findTagRel(int tag, char selectedPost){ 
        Pose3d tagPos = tagLayout.getTagPose(tag).get(); 
        double reefWidth = 65.5;
        Translation2d reefCenter = AllianceFlipUtil.apply(Reef.center);
        double x = reefCenter.getX(); // Original X coordinate
        double y = reefCenter.getY(); // Original Y coordinate
        
        // Polar coordinates (radius and angle)
        double r = 18.75 + (reefWidth / 2); // radius
        double theta = (tagPos.getRotation().toRotation2d().getRadians());
        
        double deltaX = r * Math.cos(theta); // x' component
        double deltaY = r * Math.sin(theta); // y' component
        
        // Calculate new point by translating the original point
        double newX = x + deltaX;
        double newY = y + deltaY;
        double newTheta = theta + (Math.PI/2); // in radians
        // the thing is now centered at the tag -> let's  center it on a reef now!
        // if(selectedPost.isIn(fwdArray)){
        //     moveFWD();
        // }
        // else{
        //     moveBWD();
        // }
        return new Pose2d(new Translation2d(newX, newY), new Rotation2d(Units.radiansToDegrees(theta + (Math.PI/2)))); //the extra bit makes sure  the robo is perpendicular

    }

    public int getNearestTag() {
        Pose2d relative = swerveSubsystem.odometry.getEstimatedPosition()
            .relativeTo(FieldConstants.getReefPose());

        int[] tags; // these are pretransformed to make the the logic easier
        if (FieldConstants.getAlliance() == Alliance.Red) {
            tags = new int[] {6, 7, 8, 9, 10, 11};
        } else {
            tags = new int[] {19, 18, 17,22, 21, 20};
        }   

        double angle = Math.atan2(relative.getY(), relative.getX()); 
        int index = Math.round((float) ( 
            (angle + Math.PI)
            * (6 / (2*Math.PI))
        ));

        return tags[index];
    }

    public void setSelectedTag(int tag) {
        this.selectedTag = tag;
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
