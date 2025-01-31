// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms                                                                                                                                                                
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;
  }

  public static class RobotConstants {
    public static final double robotWidthMeters = Units.inchesToMeters(25.0);
    public static final double robotLengthMeters = Units.inchesToMeters(25.0);
    
      // TODO: ############## REPLACE PLACEHOLDERS ##############
      public static final double TOTAL_MASS_KG = 60;
      public static final double MOMENT_OF_INERTIA = 5;
  }

  public static final class FieldConstants {
    public static final double GRAVITY = 9.81;
    public static final double SPEAKER_HEIGHT = 2.05; // Meters

    public static Alliance getAlliance() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get();
      }

      return Alliance.Blue;
    }
  }

  public static class SwerveModuleConstants {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEERING_GEAR_RATIO = 1.d / (150d / 7d); // 6.75:1
    // This is for L2 modules with 16T pinions
    public static final double DRIVE_GEAR_RATIO = (1.d / 6.75d);

    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double STEER_ROTATION_TO_RADIANS = STEERING_GEAR_RATIO * Math.PI * 2d;
    public static final double DRIVE_METERS_PER_MINUTE = DRIVE_ROTATION_TO_METER / 60d;
    public static final double STEER_RADIANS_PER_MINUTE = STEER_ROTATION_TO_RADIANS / 60d;

    // TODO: ############## REPLACE PLACEHOLDERS ##############
    public static final double WHEEL_FRICTION_COEFFICIENT = 1;

    // Actual drive gains
    // public static final double MODULE_KP = 0.5;
    // public static final double MODULE_KD = 0.03;

    // NOTE: This may need additional tuning!
    public static final double MODULE_KP = 0.46368;// 0.75628;// 0.7491; //.5;
    public static final double MODULE_KD = 0.0066806;// 0.0057682; //0.0076954;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 4;
    public static final int FL_STEER_ID = 4;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 4;
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(0.389893) + Math.PI * 0.5 + Math.PI;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FL_MOTOR_REVERSED = true;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 1;
    public static final int FR_STEER_ID = 1;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(0.323730) + Math.PI * 0.5 + Math.PI;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FR_MOTOR_REVERSED = true;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 2;
    public static final int BR_STEER_ID = 2;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 2;
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(-0.360107) + Math.PI * 0.5 + Math.PI;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BR_MOTOR_REVERSED = true;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 3;
    public static final int BL_STEER_ID = 3;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 3;
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(0.399902) + Math.PI * 0.5 + Math.PI;
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BL_MOTOR_REVERSED = true;

  }

  public static class DriveConstants {
    // TODO: Make sure that this is correct - this is from the SDS website but needs
    // empirical verification
    public static final double MAX_MODULE_VELOCITY = 5.21;
    public static final double MAX_ROBOT_VELOCITY = 5.21;
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    // TODO: ############## REPLACE PLACEHOLDERS ##############
    public static final double MAX_MODULE_CURRENT = 100;

    public static final double TRACK_WIDTH = Units.inchesToMeters(19.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(19.75);
    // TODO: Set this for FWERB V2
    public static final Rotation2d NAVX_ANGLE_OFFSET = Rotation2d.fromDegrees(-90);
    // TODO: I'm not going to touch this... but it seems important!
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(15);

    public static final class ModuleIndices {
      public static final int FRONT_LEFT = 0;
      public static final int FRONT_RIGHT = 2;
      public static final int REAR_LEFT = 1;
      public static final int REAR_RIGHT = 3;
    }

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double XY_SPEED_LIMIT = 1.0;
    public static final double Z_SPEED_LIMIT = 1.0;
  }

  public static class CommonConstants {
    public static final boolean LOG_INTO_FILE_ENABLED = true;
  }

  public static class Elevator {
    public static final int elevatorOnePort = 10;
    public static final int elevatorTwoPort = 11;

    public static boolean elevatorOneInverted = true;
    public static boolean elevatorTwoInverted = false;

    public static Type bottomLimitMode = Type.kNormallyOpen;


    public static double motorTurnsPerMeter = 39.44;

    public static class PID {
      public static double kP = 20.0; // 9.0;
      public static double kI = 0.0;
      public static double kD = 0.5; // 4.0;
      public static double MAX_VELOCITY = 2.8;
      public static double MAX_ACCELERATION = 18.0;
    }

    // TODO: For the first testing, set these all to zero for safety reasons
    // Remind me to pad the top and bottom of the elevator with poodles to make sure
    // we don't damage it.
    public static class Feedforward {
      public static double Ks = 0.0;
      public static double Kv = 4.0;
      public static double Ka = 0.03;
      public static double Kg = 0.1;
    }

    public static class PhysicalParameters {
      public static double gearReduction = 9.0 / 2.0;
      public static double driveRadiusMeters = 0.0182;
      public static double carriageMassKg = 1.5;
      public static double elevatorHeightMeters = Units.inchesToMeters(50.0);
      public static double elevatorBottomFromFloorMeters = Units.inchesToMeters(12.0);
      public static double elevatorCarriageHeightMeters = Units.inchesToMeters(6.0);
      public static double elevatorForwardsFromRobotCenterMeters = Units.inchesToMeters(25.0 / 2);
      public static DCMotor simMotor = DCMotor.getNeoVortex(2);
      // TODO: create maxvelocity and maxacceleration
    }
  }

  public static final class PathPlannerConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0.2);
    public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0.2);

    public static final PPHolonomicDriveController HOLONOMIC_FOLLOWER_CONTROLLER = new PPHolonomicDriveController(
      TRANSLATION_PID,
      ROTATION_PID
    );

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
      RobotConstants.TOTAL_MASS_KG,
      RobotConstants.MOMENT_OF_INERTIA,
      new ModuleConfig(
        SwerveModuleConstants.WHEEL_DIAMETER/2,
        DriveConstants.MAX_MODULE_VELOCITY,
        SwerveModuleConstants.WHEEL_FRICTION_COEFFICIENT, // TODO: ############## REPLACE PLACEHOLDERS ##############
        DCMotor.getKrakenX60(1),
        DriveConstants.MAX_MODULE_CURRENT, // TODO: ############## REPLACE PLACEHOLDERS ##############
        1
      ),
      new Translation2d(-0.5, 0.5),
      new Translation2d(0.5, 0.5),
      new Translation2d(-0.5, -0.5),
      new Translation2d(0.5, -0.5)
    );
  }

  public static final class PoseConstants {

    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevTheta = 500;
  }
}
