package frc.robot.subsystems;

import java.util.List;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;

public class SwerveSubsystem extends SubsystemBase {
    SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_RADIANS,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FL_MOTOR_REVERSED);

    SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_RADIANS,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FR_MOTOR_REVERSED);

    SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_RADIANS,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BR_MOTOR_REVERSED);

    SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_RADIANS,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BL_MOTOR_REVERSED);

    private DoubleLogEntry chassisAccelX;
    private DoubleLogEntry chassisAccelY;
    private DoubleLogEntry chassisAccelZ;
    private DoubleLogEntry chassisRotX;
    private DoubleLogEntry chassisRotY;
    private DoubleLogEntry chassisRotZ;

    PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    int[] pdh_channels = {
            18, 19,
            0, 1,
            16, 17,
            2, 3
    };

    public enum RotationStyle {
        Driver,
        AutoSpeaker,
        AutoShuttle
    }

    private RotationStyle rotationStyle = RotationStyle.Driver;

    public final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private double navxSim;

    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    private Field2d field = new Field2d();

    boolean isalliancereset = false;

    // TODO: Properly set starting pose
    public final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS,
            getRotation2d(),
            getModulePositions(), new Pose2d(), createStateStdDevs(
                    PoseConstants.kPositionStdDevX,
                    PoseConstants.kPositionStdDevY,
                    PoseConstants.kPositionStdDevTheta),
            createVisionMeasurementStdDevs(
                    PoseConstants.kVisionStdDevX,
                    PoseConstants.kVisionStdDevY,
                    PoseConstants.kVisionStdDevTheta));

    public SwerveSubsystem() {
        // ! F
        // zeroHeading()

        // --------- Path Planner Init ---------- \\

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforward) -> setChassisSpeedsAUTO(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                PathPlannerConstants.HOLONOMIC_FOLLOWER_CONTROLLER,
                PathPlannerConstants.ROBOT_CONFIG,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }

                    return false;

                },
                this // Reference to this subsystem to set requirements
        );

        NamedCommands.registerCommand("namedCommand", new PrintCommand("Ran namedCommand"));

        chassisAccelX = new DoubleLogEntry(DataLogManager.getLog(), "Chassis/acceleration/x");
        chassisAccelY = new DoubleLogEntry(DataLogManager.getLog(), "Chassis/acceleration/y");
        chassisAccelZ = new DoubleLogEntry(DataLogManager.getLog(), "Chassis/acceleration/z");
        chassisRotX = new DoubleLogEntry(DataLogManager.getLog(), "Chassis/rot_speed/x");
        chassisRotY = new DoubleLogEntry(DataLogManager.getLog(), "Chassis/rot_speed/y");
        chassisRotZ = new DoubleLogEntry(DataLogManager.getLog(), "Chassis/rot_speed/z");
    }

    @Override
    public void periodic() {


        if ((!isalliancereset && DriverStation.getAlliance().isPresent())) {
            RobotContainer.LLContainer.estimateMT1OdometryPrelim(odometry, lastChassisSpeeds, navX, getModulePositions());
            SmartDashboard.putString("Prelim odometry position", odometry.getEstimatedPosition().toString());
            isalliancereset = true;
        }

        RobotContainer.LLContainer.estimateMT1Odometry(odometry, lastChassisSpeeds, navX);

        //odometry.update(getRotation2d(), getModulePositions());

                // if (DriverStation.getAlliance().isPresent()) {
        // switch (DriverStation.getAlliance().get()) {
        // case Red:
        // field.setRobotPose(new Pose2d(new Translation2d(16.5 - getPose().getX(),
        // getPose().getY()),
        // getPose().getRotation()));
        // break;

        // case Blue:
        // field.setRobotPose(getPose());
        // break;
        // }
        // } else {
        // // If no alliance provided, just go with blue
        field.setRobotPose(getPose());
        // }

        SmartDashboard.putData("Field", field);

        SmartDashboard.putString("Robot Pose",
                getPose().toString());
        // double swerveCurrent = 0;
        // for (int chan : pdh_channels)
        // swerveCurrent += pdh.getCurrent(chan);
        // SmartDashboard.putNumber("SwerveSubsystem Amps", swerveCurrent);
        // SmartDashboard.putNumber("PDH Amps", pdh.getTotalCurrent());

        SmartDashboard.putNumberArray("SwerveStates", new double[] {
                frontLeft.getModuleState().angle.getDegrees() + 90, -frontLeft.getModuleState().speedMetersPerSecond,
                frontRight.getModuleState().angle.getDegrees() + 90, -frontRight.getModuleState().speedMetersPerSecond,
                backLeft.getModuleState().angle.getDegrees() + 90, -backLeft.getModuleState().speedMetersPerSecond,
                backRight.getModuleState().angle.getDegrees() + 90, -backRight.getModuleState().speedMetersPerSecond
        });

        chassisAccelX.append(navX.getRawAccelX());
        chassisAccelY.append(navX.getRawAccelY());
        chassisAccelZ.append(navX.getRawAccelZ());
        chassisRotX.append(navX.getRawGyroX());
        chassisRotY.append(navX.getRawGyroY());
        chassisRotZ.append(navX.getRawGyroZ());
    }

    public void zeroHeading() {
        setHeading(0);
    }

    public void setHeading(double deg) {
        if (Robot.isSimulation()) {
            navxSim = Units.degreesToRadians(deg);
        }
        // navX.reset();
        // navX.setAngleAdjustment(deg);

        double error = deg - navX.getAngle();
        double new_adjustment = navX.getAngleAdjustment() + error;
        navX.setAngleAdjustment(new_adjustment);
    }

    public Pose2d getPose() {
        Pose2d p = odometry.getEstimatedPosition();
        return p;
    }

    public void resetOdometry(Pose2d pose) {
        // TODO: TEST
        setHeading(Units.radiansToDegrees(pose.getRotation().times(-1.0).getRadians()
                + (FieldConstants.getAlliance() == Alliance.Red ? Math.PI : 0.0)));

        SmartDashboard.putNumber("HEading reset to", getHeading());
        SmartDashboard.putBoolean("HASBEENREET", true);
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getHeading() {
        return Robot.isSimulation() ? -navxSim : Units.degreesToRadians(Math.IEEEremainder(-navX.getAngle(), 360));
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    public void stopDrive() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] states) {
        lastChassisSpeeds = DriveConstants.KINEMATICS.toChassisSpeeds(states);
        // Normalize speeds so they are all obtainable
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_MODULE_VELOCITY);
        frontLeft.setModuleState(states[Constants.DriveConstants.ModuleIndices.FRONT_LEFT]);
        frontRight.setModuleState(states[Constants.DriveConstants.ModuleIndices.FRONT_RIGHT]);
        backRight.setModuleState(states[Constants.DriveConstants.ModuleIndices.REAR_RIGHT]);
        backLeft.setModuleState(states[Constants.DriveConstants.ModuleIndices.REAR_LEFT]);
    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp;
        tmp = speeds.omegaRadiansPerSecond;
        speeds.omegaRadiansPerSecond *= -1;
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DriveConstants.KINEMATICS.toChassisSpeeds(
                frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(),
                backRight.getModuleState());

        return Robot.isSimulation() ? lastChassisSpeeds : speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = {
                frontLeft.getModulePosition(),
                frontRight.getModulePosition(),
                backLeft.getModulePosition(),
                backRight.getModulePosition()
        };

        return states;
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulate_step();
        frontRight.simulate_step();
        backLeft.simulate_step();
        backRight.simulate_step();
        navxSim += 0.02 * lastChassisSpeeds.omegaRadiansPerSecond;
    }

    public RotationStyle getRotationStyle() {
        return rotationStyle;
    }

    public void setRotationStyle(RotationStyle style) {
        rotationStyle = style;
    }

    // ---------- Path Planner Methods ---------- \\

    public Command loadPath(String name) {
        return new PathPlannerAuto(name);
    }

    public Command followPathCommand(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforward) -> setChassisSpeedsAUTO(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                PathPlannerConstants.HOLONOMIC_FOLLOWER_CONTROLLER,
                PathPlannerConstants.ROBOT_CONFIG,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception exception) {
            return Commands.none();
        }
    }

    public PathPlannerPath generateOTFPath(PathPoint... pathPoints) {
        // Create the path using the bezier points created above
        PathPlannerPath path = PathPlannerPath.fromPathPoints(
                List.of(pathPoints),
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a
                                                                         // differential drivetrain, the angular
                                                                         // constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation
                                                                   // here. If using a differential drivetrain, the
                                                                   // rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }

    public void updateVisionOdometry() {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            // if (mt1.rawFiducials[0].distToCamera > 3) {
            if (mt1.rawFiducials[0].distToCamera > 5) { // TODO: TUNE!!!

                doRejectUpdate = true;
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            // odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            odometry.setVisionMeasurementStdDevs(createVisionMeasurementStdDevs(
                    PoseConstants.kVisionStdDevX,
                    PoseConstants.kVisionStdDevY,
                    PoseConstants.kVisionStdDevTheta));
            odometry.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
        }
    }

    public void updateMegaTagOdometry() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", odometry.getEstimatedPosition().getRotation().getDegrees(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (Math.abs(navX.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                            // vision updates
        {
            doRejectUpdate = true;
        }

        if (mt2.tagCount <= 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            // odometry.setVisionMeasurementStdDevs(VecBuilder.fill(2,2,2.0*PoseConstants.kVisionStdDevTheta));
            odometry.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 9999999));

            odometry.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }
    }

    public Vector<N3> createStateStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }

    public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }
}
