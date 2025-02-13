// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class LimelightContainer {
  static int SIMCOUNTER = 0;
  static int RLCOUNTER = 0;
  static int RLCountermt1 = 0;
  private static ArrayList<Limelight> limelights = new ArrayList<Limelight>();


  public static boolean isOnLeft(){
    int[] validIds = new int[6]; //change these to be all reef tags, for given alliance (check if they red or blue!!)
    
    if(FieldConstants.getAlliance() == Alliance.Blue){
        for(int i = 0; i < 6; i++){
        validIds[i] = i+17;
        }
    }
    else if(FieldConstants.getAlliance() == Alliance.Red){
        for(int i = 0; i < 6; i++){
            validIds[i] = i +6;
        }
    }

    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-fl", validIds);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-fr", validIds);


    if(LimelightHelpers.getTA("limelight-fl")>1){
      int[] allIDs = new int[22];
      for(int i = 1; i <= 22; i++){
          allIDs[i-1] = i; // sets allIDs to be {1, 2, 3, etc.}
      }
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight-fl", allIDs);
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight-fr", allIDs);
        return true;
    }
    else if(LimelightHelpers.getTA("limelight-fr")>1){
      int[] allIDs = new int[22];
      for(int i = 1; i <= 22; i++){
          allIDs[i-1] = i; // sets allIDs to be {1, 2, 3, etc.}
      }
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight-fl", allIDs);
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight-fr", allIDs);

        return false;
    }
    else return false;
    
  }

  public LimelightContainer(Limelight... limelights) {
    for (Limelight limelight : limelights) {
      LimelightContainer.limelights.add(limelight);
    }
    enableLimelights(true);
  }

  public void enableLimelights(boolean enable) {
    for (Limelight limelight : limelights) {
      if(limelight != null){
        limelight.setEnabled(enable);
      }
    }
  }

  public static void estimateSimMT1() {
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());
      if (mt1 == null) { // in case not all limelights are connected
        continue;
      }
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        SmartDashboard.putString("Simulated Pos MT1", mt1.pose.toString() + SIMCOUNTER);
        SIMCOUNTER++;
      }
    }
  }

  public void estimateMT1OdometryPrelim(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, AHRS navx,
      SwerveModulePosition[] swerveModulePositions) {
    int framesChecked = 0;
    ArrayList<Pose2d> validPoses = new ArrayList<>();

    for (int i = 0; (i < 100) && (framesChecked < 10); i++) {
      for (Limelight limelight : limelights) {

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

        if (mt1 == null) {
          continue;
        }

        double currtime = mt1.timestampSeconds;
        if (currtime != limelight.getLastFrameTime()) {
          framesChecked++;
          boolean doRejectUpdate = false;

          if (mt1.tagCount == 0) {
            doRejectUpdate = true;
          }

          if (Math.abs(navx.getRate()) > 720) {
            doRejectUpdate = true;
          }

          if (!doRejectUpdate) {
            validPoses.add(mt1.pose);
            SmartDashboard.putString("Pos MT1 prelim: ", mt1.pose.toString() + " " + RLCountermt1);
          }

          RLCountermt1++;
          limelight.setLastFrame(currtime);
        }
      }
    }

    if (!validPoses.isEmpty()) {
      double avgX = 0, avgY = 0, avgRotation = 0;
      for (Pose2d pose : validPoses) {
        avgX += pose.getX();
        avgY += pose.getY();
        avgRotation += pose.getRotation().getDegrees();
      }
      avgX /= validPoses.size();
      avgY /= validPoses.size();
      avgRotation /= validPoses.size();

      odometry.resetPosition(Rotation2d.fromDegrees(avgRotation), swerveModulePositions, new Pose2d(avgX, avgY, Rotation2d.fromDegrees(avgRotation)));
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

      if (Math.abs(navx.getRate()) > 720) {
        doRejectUpdate = true;
      }
      
      if (Math.abs(odometry.getEstimatedPosition().getX() - mt1.pose.getX()) > .4) {
        doRejectUpdate = true; 
        SmartDashboard.putBoolean("Rejected due to too-far pose", true);
       }

      if (!doRejectUpdate) {
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.9, .9, .2));
        odometry.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }

      RLCountermt1++;
    }
  }

  public void estimateMT2Odometry(SwerveDrivePoseEstimator odometry, ChassisSpeeds speeds, AHRS navx) {

    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;


      LimelightHelpers.SetRobotOrientation(limelight.getName(),
          odometry.getEstimatedPosition().getRotation().getDegrees(), navx.getRate(), 0, 0, 0, 0);


      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());

      if (mt2 == null) {
        continue;
      }

      if (Math.abs(navx.getRate()) > 720) {
        doRejectUpdate = true;
      }

      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (mt2.pose.getX() == 8.77 && mt2.pose.getY() == 4.03) {
        doRejectUpdate = true;
      }

      if (Math.abs(odometry.getEstimatedPosition().getX() - mt2.pose.getX()) > .4) {
        doRejectUpdate = true; 
        SmartDashboard.putBoolean("Rejected due to too-far pose", true);
      }

      if (!doRejectUpdate) {

        SmartDashboard.putString("Pos (mt2): ", mt2.pose.toString() + " " + RLCOUNTER);

        if (mt2.tagCount == 1) {
          odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.4, .4, .3));
          odometry.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
        } else if (mt2.tagCount >= 2) {
          odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.1, .1, .1));
          odometry.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);

        }

        SmartDashboard.putNumber("Dist", mt2.avgTagDist);
      }
      RLCOUNTER++;
    }
  }

}
