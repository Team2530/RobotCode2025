// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms                                                                                                                                                                
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
    public static final double TOTAL_MASS_KG = 10;
    public static final double MOMENT_OF_INERTIA = 1;
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
    public static final double STEERING_GEAR_RATIO = 1.d / (150d / 7d);
    // This is for L2 modules with 16T pinions
    public static final double DRIVE_GEAR_RATIO = (1.d / 6.75d) * (16.f / 14.f);

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
    public static final int FL_DRIVE_ID = 34;
    public static final int FL_STEER_ID = 4;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 54;
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(-0.310303) + Math.PI * 0.5 + Math.PI;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FL_MOTOR_REVERSED = true;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 31;
    public static final int FR_STEER_ID = 1;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 51;
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(-0.253906) + Math.PI * 0.5 + Math.PI;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FR_MOTOR_REVERSED = true;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 32;
    public static final int BR_STEER_ID = 2;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 52;
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(0.353027) + Math.PI * 0.5 + Math.PI;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BR_MOTOR_REVERSED = true;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 33;
    public static final int BL_STEER_ID = 3;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 53;
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(-0.134033) + Math.PI * 0.5 + Math.PI;
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
    public static final double MAX_MODULE_CURRENT = 30;

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

  // TODO: ##################### PLACEHOLDERS #####################
  public static class Climber {
    public static final int MOTOR_PORT = 20;
    public static final ProfiledPIDController PID = new ProfiledPIDController(
        1,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE));

    public static boolean DBG_DISABLED = false;
    public static double GEAR_RATIO = 45.0;

    public static double DEPLOY_SOFT_LIMIT = -6.0;
    public static double CLIMB_SOFT_LIMIT = -3.10;
  }

  // TODO: ##################### PLACEHOLDERS #####################
  public static class Coral {
    public static int LEFT_ULTRASONIC_PORT = 0;
    public static int RIGHT_ULTRASONIC_PORT = 1;

    public static boolean DEBUG_PIDS = true;

    public static class Pivot {
      public static final int MOTOR_PORT = 14;
      public static final int ENCODER_PORT = 28;
      public static boolean DBG_DISABLED = false;

      public static final boolean ENCODER_INVERTED = false;

      public static final double MAXIMUM_ANGLE = Units.degreesToRadians(80);
      public static final double ELEVATOR_BORDER_ANGLE = Units.degreesToRadians(10);

      // TODO: Tune in simulation
      public static final ProfiledPIDController PID = new ProfiledPIDController(
          12.0,
          0.0,
          0.005,
          new TrapezoidProfile.Constraints(7.0, 15.0));

      // Updated with THEORETICAL values
      public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(
          0.0,
          0.0, // V
          0.0, // 1.0, // V*s/rad
          0.00// V*s^2/rad
      );

      public static class PhysicalConstants {
        public static final DCMotor MOTOR = DCMotor.getNeoVortex(1);
        public static final double NET_REDUCTION = 96.0;
        public static final double MASS_KG = 4.7727;
        public static final double ARM_LENGTH_METERS = 0.510;
        public static final double JOINT_LENGTH_METERS = Units.inchesToMeters(23.0);
        public static final double MOI = 0.2875548495; // Kg*m^2
      }
    }

    public static class Roll {
      public static final int MOTOR_PORT = 15;
      public static final boolean MOTOR_INVERTED = false;
      public static final boolean ENCODER_INVERTED = false;
      public static final double ENCODER_OFFSET_VOLTS = -1.85;
      public static boolean DBG_DISABLED = false;

      public static final ProfiledPIDController PID = new ProfiledPIDController(
          6,
          0.0,
          0.02,
          new TrapezoidProfile.Constraints(15, 25.0));
      // public static final SimpleMotorFeedforward FEEDFORWARD = new
      // SimpleMotorFeedforward(0.0, 0.1);

      public static final double MAXIMUM_ANGLE = Units.degreesToRadians(90);

      public static class PhysicalConstants {
        public static DCMotor MOTOR = DCMotor.getNeo550(1);
        public static final double NET_REDUCTION = 45.0;
        public static final double MASS_KG = 2.85; // Includes a coral
        public static final double ARM_LENGTH_METERS = 0.083;
        public static final double JOINT_LENGTH_METERS = 0.10;
        public static final double MOI = 0.0403605447; // Kg*m^2
      }
    }

    public static class Pitch {
      public static final int MOTOR_PORT = 16;
      public static final boolean MOTOR_INVERTED = true;
      public static final boolean ENCODER_INVERTED = true;
      public static final double ENCODER_OFFSET_VOLTS = -2.7;
      public static boolean DBG_DISABLED = false;

      public static final double MAXIMUM_ANGLE = Units.degreesToRadians(115.0);

      public static final ProfiledPIDController PID = new ProfiledPIDController(
          7.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(10.0, 30.0)); // Radians

      // public static final SimpleMotorFeedforward FEEDFORWARD = new
      // SimpleMotorFeedforward(0.0, 0.45);

      public static class PhysicalConstants {
        public static DCMotor MOTOR = DCMotor.getNeo550(1);
        public static final double NET_REDUCTION = 92.85714286; // Yeah this is cursed
        public static final double MASS_KG = 2.16; // Includes a coral
        public static final double ARM_LENGTH_METERS = 0.101;
        public static final double JOINT_LENGTH_METERS = Units.inchesToMeters(13.875);
        public static final double MOI = 0.00200055915; // Kg*m^2
      }
    }

    public static class Intake {
      public static final int MOTOR_PORT = 17;
      public static boolean DBG_DISABLED = false;
      public static boolean MOTOR_INVERTED = true;

      public static final double POSITIVE_RATE_LIMIT = 200.0; // Fast shoot
      public static final double NEGATIVE_RATE_LIMIT = 100.0; // Slow intake

      public static final double SCORE_EXTRA_SECONDS = 1.0;

      public static final double CURRENT_LIMIT = 12.5; // Stator

      public static final class PhysicalConstants {
        public static final DCMotor MOTOR = DCMotor.getFalcon500(1);
        public static final double MOI = 0.1; // J*KG / M^2
        public static final double GEARING = 1;
      }
    }
  }

  // TODO: ##################### PLACEHOLDERS #####################
  public static final class Algae {
    public static final class Pivot {
      public static final int MOTOR_PORT = 18;
      public static final int ENCODER_PORT = 27;

      public static final ProfiledPIDController PID = new ProfiledPIDController(
          1,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE));
      public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(
          1,
          1,
          1);

      public static final double RETRACTED_LIMIT_DEGREES = 10.0;
      public static final double EXTENDED_LIMIT_DEGREES = 90.0;

      public static class PhysicalConstants {
        public static final DCMotor MOTOR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 1; // TODO: this is the only placeholder
        public static final double NET_REDUCTION = 42.66666667; // Yeah this is cursed
        public static final double MASS_KG = 3.18;
        public static final double ARM_LENGTH_METERS = 0.2349863728;
        public static final double MOI = 0.1114866914; // Kg*m^2
      }
    }

    public static final class Intake {
      public static final int MOTOR_PORT = 19;
      public static final int BEAMBREAK_PORT = 0; // NOTE: Beambreak will *probably* be a rockwell proximity sensor
                                                  // wired into the SPARK max

      public static final double POSITIVE_RATE_LIMIT = 5.0;
      public static final double NEGATIVE_RATE_LIMIT = -5.0;

      public static final class PhysicalConstants {
        public static final DCMotor MOTOR = DCMotor.getNeo550(1);
        public static final double MOI = 0.1; // J*KG / M^2
        public static final double GEARING = 1;
      }
    }
  }

  // See
  // https://cad.onshape.com/documents/fa9a0365dfdf7e376f93f1b4/w/36bfb0cc9de95ef5933791e3/e/700ba3cf920578fe61d3ec24
  public static final class Elevator {
    public static final class Leader {
      public static final int MOTOR_PORT = 10;
      public static final boolean INVERTED = false;
    }

    public static final class Follower {
      public static final int MOTOR_PORT = 11;
      public static final boolean INVERTED = true;
    }

    public static boolean DBG_DISABLED = false;

    public static double MOTOR_REVOLUTIONS_PER_METER = 32.81;

    // TODO: Tune! Use FWERB for now
    public static class PID {
      public static double kP = 28.0;
      public static double kI = 0.0;
      public static double kD = 0.01;
      public static double MAX_VELOCITY = 2.80;
      // TODO: Needs empirical testing - analyze setpoint v/s state graphs to see if
      // the elevator can make or exceed this
      public static double MAX_ACCELERATION = 15.0;
    }

    // TODO: PAD THE ELEVATOR!!!!!!!
    public static class FeedforwardConstants {
      public static double Ks = 0.0;
      public static double Kv = 2.45

      ;
      public static double Ka = 0.1;
      public static double Kg = 0.05; // TODO: Check this!!!
    }

    public static ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(
        FeedforwardConstants.Ks,
        FeedforwardConstants.Kg,
        FeedforwardConstants.Kv,
        FeedforwardConstants.Ka);

    public static class PhysicalParameters {
      public static final double GEARING = 5.0 / 2.0;
      public static final double DRIVE_RADIUS_METERS = 0.0121;
      public static final double CARRIAGE_MASS_KG = 6.0; // Load on the SECOND stage NOTE: This includes the weight
                                                         // "reduction" due to CF spring counterbalance!

      public static final double MAX_TRAVEL = Units.inchesToMeters(59.5);
      public static final double BOTTOM_TO_FLOOR = Units.inchesToMeters(3.0); // Relative to bottom of stage 2
      public static final double CARRIAGE_HEIGHT = Units.inchesToMeters(8.0); // Bottom to top of stage 2

      public static final double CORAL_PIVOT_VERTICAL_OFFSET = Units.inchesToMeters(6.0); // From bottom of stage 2 to
                                                                                          // coral arm pivot axis
      public static final double CORAL_PIVOT_HORIZONTAL_OFFSET = Units.inchesToMeters(7.5); // From bottom of stage 2 to
                                                                                            // coral arm pivot axis
      public static final double ELEVATOR_FORWARDS_OFFSET = Units.inchesToMeters(1); // To mid-plane of elevator

      public static final DCMotor MOTOR = DCMotor.getNeoVortex(2);
    }
  }

  public static final class PathPlannerConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0.2);
    public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0.2);

    public static final PPHolonomicDriveController HOLONOMIC_FOLLOWER_CONTROLLER = new PPHolonomicDriveController(
        TRANSLATION_PID,
        ROTATION_PID);

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
        RobotConstants.TOTAL_MASS_KG,
        RobotConstants.MOMENT_OF_INERTIA,
        new ModuleConfig(
            SwerveModuleConstants.WHEEL_DIAMETER / 2,
            DriveConstants.MAX_MODULE_VELOCITY,
            SwerveModuleConstants.WHEEL_FRICTION_COEFFICIENT, // TODO: ############## REPLACE PLACEHOLDERS
                                                              // ##############
            DCMotor.getKrakenX60(1),
            DriveConstants.MAX_MODULE_CURRENT, // TODO: ############## REPLACE PLACEHOLDERS ##############
            1),
        DriveConstants.KINEMATICS.getModules());
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
