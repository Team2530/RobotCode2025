package frc.robot.commands.coral.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import frc.robot.subsystems.coral.CoralSubsystem.MirrorPresets;

public class WristAlignAssistManual extends Command {
    CoralSubsystem coralSubsystem;
    CoralPresets startingPreset;
    XboxController operatorController;
    SwerveSubsystem swerveSubsystem;

    public WristAlignAssistManual(CoralSubsystem coralSub, XboxController operatorController,
            SwerveSubsystem swerveSubsystem) {
        coralSubsystem = coralSub;
        this.operatorController = operatorController;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        startingPreset = coralSubsystem.getCurrentPreset();
    }

    @Override
    public void execute() {
        // If doing "full manual" adjust, multiply by (coralSubsystem.getMirror() ==
        // MirrorPresets.LEFT ? 1.0 : -1.0) for consistent rotation direction
        double adjustCommandRight = operatorController.getRightX();

        Rotation2d driverStationRight = Rotation2d.fromDegrees(-90);
        if (FieldConstants.getAlliance() == Alliance.Red)
            driverStationRight = driverStationRight.unaryMinus();

        boolean directionFlip = (swerveSubsystem.getOdometryPose().getRotation().minus(driverStationRight))
                .getCos() > 0.0;

        double adjustCommandForwards = -adjustCommandRight * (directionFlip ? -1.0 : 1.0);

        coralSubsystem
                .setCustomRollDegrees(
                        (coralSubsystem.getMirror() == MirrorPresets.LEFT ? -1.0 : 1.0) * MathUtil.clamp(
                                (90.0 - Units.radiansToDegrees(
                                        Math.atan2(adjustCommandForwards, 2.0))),
                                90 - 30.0,
                                90 + 30.0));
    }

    @Override
    public boolean isFinished() {
        return !(startingPreset == CoralPresets.LEVEL_4 || startingPreset == CoralPresets.LEVEL_2
                || startingPreset == CoralPresets.LEVEL_3);
    }

    @Override
    public void end(boolean interrupted) {
        // Restore roll setting
        coralSubsystem.setCoralPresetRoll(startingPreset);
    }
}
