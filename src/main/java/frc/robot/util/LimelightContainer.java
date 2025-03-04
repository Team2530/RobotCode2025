// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PoseConstants;

public class LimelightContainer {
  static int SIMCOUNTER = 0;
  static int RLCOUNTER = 0;
  static int RLCountermt1 = 0;
  private static ArrayList<Limelight> limelights = new ArrayList<Limelight>();

  public LimelightContainer(Limelight... limelights) {
    for (Limelight limelight : limelights) {
      LimelightContainer.limelights.add(limelight);
      LimelightHelpers.SetIMUMode(limelight.getName(), 0);
    }
    enableLimelights(true);
    
  }

  public void enableLimelights(boolean enable) {
    for (Limelight limelight : limelights) {
      limelight.setEnabled(enable);
    }
  }

  public static void estimateSimOdometry() {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
      if (mt2 == null) { // in case not all limelights are connected
        continue;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        SmartDashboard.putString("Simulated Pos", mt2.pose.toString() + SIMCOUNTER);
        SIMCOUNTER++;
      }
    }
  }

  public void estimateMT1OdometryPrelim(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, AHRS navx,
      SwerveModulePosition[] swerveModulePositions) {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;

      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

      if (mt1 == null) {
        continue;
      }

      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (Math.abs(navx.getRate()) > 720) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {

        // odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.2, .2, .2));
        // odometry.addVisionMeasurement(
        // mt1.pose,
        // mt1.timestampSeconds);

        // odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.2, .2, 20));
        // odometry.addVisionMeasurement(
        // mt1.pose,
        // mt1.timestampSeconds);

        odometry.resetPosition(mt1.pose.getRotation(), swerveModulePositions, mt1.pose);

        SmartDashboard.putString("Pos MT1 prelim: ", mt1.pose.toString() + " " + RLCountermt1);
      }

      RLCountermt1++;
    }
  }

  public void estimateMT1Odometry(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, AHRS navx) {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;

      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

      if (mt1 == null) {
        continue;
      }

      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if(mt1.avgTagDist < Units.feetToMeters(10)){
        doRejectUpdate = true;
      }

      if (Math.abs(navx.getRate()) > 720) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {

        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(3, 3, 9999));
        odometry.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);

        SmartDashboard.putString("Pos MT1: ", mt1.pose.toString() + " " + RLCountermt1);

        limelight.pushPoseToShuffleboard(limelight.getName() + "mt1", mt1.pose);
      }

      RLCountermt1++;
    }
  }

  public void estimateMT2Odometry(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, AHRS navx) {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(limelight.getName(), navx.getAngle(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
      if(Math.abs(navx.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        limelight.pushPoseToShuffleboard(limelight.getName() + "mt2", mt2.pose);
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(3,3,9999999));
        odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

  public int findNearestTagPos(SwerveDrivePoseEstimator odometry) {
    double min = 43490824;
    int toReturn = 0;
    for (int i = 0; i < 22; i++) {
      double thing = Math.sqrt(Math.abs(PoseConstants.tagPoses.get(i).getX() - odometry.getEstimatedPosition().getX()));
      if (thing < min) {
        min = thing;
        toReturn = i + 1;
      }
    }
    return toReturn;
  }

  
}