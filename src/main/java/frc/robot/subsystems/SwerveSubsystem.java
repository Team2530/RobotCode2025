package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.RobotContainer;
import frc.robot.Robot;

@Logged
public class SwerveSubsystem extends SubsystemBase {

    boolean isalliancereset = false;

    SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_RADIANS,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FL_MOTOR_REVERSED,
            SwerveModuleConstants.FL_STEERING_MOTOR_REVERSED);

    SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_RADIANS,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FR_MOTOR_REVERSED,
            SwerveModuleConstants.FR_STEERING_MOTOR_REVERSED);

    SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_RADIANS,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BR_MOTOR_REVERSED,
            SwerveModuleConstants.BR_STEERING_MOTOR_REVERSED);

    SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_RADIANS,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BL_MOTOR_REVERSED,
            SwerveModuleConstants.BL_STEERING_MOTOR_REVERSED);

    public final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private double navxSim;

    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    private Field2d field = new Field2d();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Odometry Pose", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> swerveStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> swerveTargetStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve Target States", SwerveModuleState.struct).publish();

    // TODO: Properly set starting pose
    public final SwerveDrivePoseEstimator odometry;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public SwerveSubsystem() {
        odometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS,
                getGyroRotation2d(),
                getModulePositions(), new Pose2d(), createStateStdDevs(
                        PoseConstants.kPositionStdDevX,
                        PoseConstants.kPositionStdDevY,
                        PoseConstants.kPositionStdDevTheta),
                createVisionMeasurementStdDevs(
                        PoseConstants.kVisionStdDevX,
                        PoseConstants.kVisionStdDevY,
                        PoseConstants.kVisionStdDevTheta));

        // --------- Path Planner Init ---------- \\
        RobotConfig config = Constants.PathPlannerConstants.ROBOT_CONFIG;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        NamedCommands.registerCommand("namedCommand", new PrintCommand("Ran namedCommand"));

        setpointGenerator = new SwerveSetpointGenerator(
                config,
                Constants.SwerveModuleConstants.STEER_MAX_RAD_SEC);

        previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(),
                DriveFeedforwards.zeros(config.numModules));
    }

    public void configurePathplanner() {
        RobotConfig config = Constants.PathPlannerConstants.ROBOT_CONFIG;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getOdometryPose, // Robot pose supplier
                this::resetOdometryAndGyro, // Method to reset odometry (will be called if your auto has a starting
                                            // pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforward) -> {
                    SmartDashboard.putString("PP FF", feedforward.toString());
                    setChassisSpeedsAuto(speeds);
                }, // Method that will drive the robot given ROBOT
                   // RELATIVE ChassisSpeeds
                Constants.PathPlannerConstants.HOLONOMIC_FOLLOWER_CONTROLLER,
                // Constants.PathPlannerConstants.ROBOT_CONFIG, // The robot configuration
                config,
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
    }

    @Override
    public void periodic() {
        // if ((!isalliancereset && DriverStation.getAlliance().isPresent())) {
        //     RobotContainer.LLContainer.estimateMT1OdometryPrelim(odometry,
        //     lastChassisSpeeds, navX, getModulePositions());
        //     SmartDashboard.putString("Prelim odometry position",
        //     odometry.getEstimatedPosition().toString());
        //     isalliancereset = true;
        // }

        RobotContainer.LLContainer.estimateMT1Odometry(odometry, lastChassisSpeeds,
            navX);

        odometry.update(getGyroRotation2d(), getModulePositions());
        SmartDashboard.putString("Odometry current pos", getOdometryPose().toString());

        field.setRobotPose(getOdometryPose());
        posePublisher.set(getOdometryPose());

        SmartDashboard.putData("Field", field);
        swerveStatesPublisher.set(getModuleStates());
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

    public Pose2d getOdometryPose() {
        Pose2d p = odometry.getEstimatedPosition();
        return p;
    }

    public void resetOdometry(Pose2d pose) {
        // setHeading(Units.radiansToDegrees(pose.getRotation().times(-1.0).getRadians()
        // + (FieldConstants.getAlliance() == Alliance.Red ? Math.PI : 0.0)));
        // SmartDashboard.putNumber("HEading reset to", getGyroHeading());
        // SmartDashboard.putBoolean("HASBEENREET", true);
        odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometryAndGyro(Pose2d pose) {
        setHeading(Units.radiansToDegrees(pose.getRotation().times(-1.0).getRadians()
                + (FieldConstants.getAlliance() == Alliance.Red ? Math.PI : 0.0)));
        resetOdometry(pose);
    }

    public double getGyroHeading() {
        return Robot.isSimulation() ? navxSim : Units.degreesToRadians(Math.IEEEremainder(-navX.getAngle(), 360));
    }

    public Rotation2d getGyroRotation2d() {
        return new Rotation2d(getGyroHeading());
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
        frontLeft.setModuleState(states[0]);
        frontRight.setModuleState(states[1]);
        backLeft.setModuleState(states[2]);
        backRight.setModuleState(states[3]);
        swerveTargetStatesPublisher.set(states);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setChassisSpeedsAuto(ChassisSpeeds speeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint,
                speeds,
                0.02);
        setModules(previousSetpoint.moduleStates());
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

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
                frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(),
                backRight.getModuleState()
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

    // ---------- Path Planner Methods ---------- \\

    public Command loadPath(String name) {
        return new PathPlannerAuto(name);
    }

    public Vector<N3> createStateStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }

    public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }
}