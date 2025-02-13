// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.algae.ShootAlgaeCommand;
import frc.robot.commands.coral.IntakeCoralCommand;
import frc.robot.commands.coral.PurgeCoralIntakeCommand;
import frc.robot.commands.coral.ScoreCoralCommand;
import frc.robot.commands.coral.motion.MoveElevator;
import frc.robot.commands.coral.motion.MovePitch;
import frc.robot.commands.coral.motion.MovePivot;
import frc.robot.commands.coral.motion.MoveRoll;
import frc.robot.commands.coral.motion.StowArm;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.*;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.util.LimelightContainer;
import frc.robot.subsystems.Limelight.LimelightType;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private static final Limelight LL_FR = new Limelight(LimelightType.LL4, "limelight-fr", true, true);
    private static final Limelight LL_FL = new Limelight(LimelightType.LL4, "limelight-fl", true, true);
    private static final Limelight LL_BR  = new Limelight(LimelightType.LL4, "limelight-br", true, true);
    private static final Limelight LL_BL = new Limelight(LimelightType.LL4, "limelight-bl", true, true);

    public static final LimelightContainer LLContainer = new LimelightContainer(LL_FR, LL_FL, LL_BR, LL_BL);



    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController debugXboxController = new CommandXboxController(3);

    // private final CommandXboxController debugXbox = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;

    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    private final CoralSubsystem coralSubsystem = new CoralSubsystem();

    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        DataLogManager.logNetworkTables(true);
        DataLogManager.start();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
    }

    private CoralPresets selectedScoringPreset = CoralPresets.STOW;
    private CoralPresets lockedPreset = CoralPresets.STOW;
    private boolean isScoring = false;

    Trigger coralAquisition = new Trigger(coralSubsystem.isHoldingSupplier());
    Trigger coralInPosition = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean() {
            return coralSubsystem.isSupposedToBeInPosition();
        };
    });

    private void lockCoralArmPreset(CoralPresets preset) {
        lockedPreset = preset;
        SmartDashboard.putString("Locked Coral Preset", preset.toString());
    }

    private Supplier<CoralPresets> currentLockedPresetSupplier = new Supplier<CoralSubsystem.CoralPresets>() {
        public CoralPresets get() {
            return lockedPreset;
        };
    };

    // go to preset position:
    // 1. Stow arm & wrist
    // 2. Move elevator
    // 3. Deploy arm
    // 4. Rotate wrist
    private Command getGoToLockedPresetCommand() {
        return new InstantCommand(() -> {
            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        }).andThen(new StowArm(coralSubsystem))
                .andThen(new MoveElevator(coralSubsystem, currentLockedPresetSupplier))
                .andThen(new ParallelCommandGroup(
                        new MovePivot(coralSubsystem, currentLockedPresetSupplier),
                        new MoveRoll(coralSubsystem, currentLockedPresetSupplier)))
                .andThen(new MovePitch(coralSubsystem, currentLockedPresetSupplier))
                .andThen(new InstantCommand(() -> {
                    SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString() + " - Done");
                }));
    }

    private Command getStowCommand() {
        return new InstantCommand(() -> {
            lockCoralArmPreset(CoralPresets.STOW);
        }).andThen(getGoToLockedPresetCommand());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        /*
         * operator
         */
        // low algae TODO: Make a preset for low reef algae
        // operatorXbox.leftBumper().onTrue(new InstantCommand(() -> {
        // algaeSubsystem.setAlgaePreset(AlgaePresets.FLOOR);
        // }));
        // // high algae TODO: Make a preset for high reef algae
        // operatorXbox.rightBumper().onTrue(new InstantCommand(() -> {
        // algaeSubsystem.setAlgaePreset(AlgaePresets.STOW);
        // }));

        // L1
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_1;
        }));
        // L2
        operatorXbox.x().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_2;
        }));
        // L3
        operatorXbox.y().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_3;
        }));
        // L4
        operatorXbox.b().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_4;
        }));

        // Score!!!
        operatorXbox.rightTrigger().whileTrue((new InstantCommand(() -> {
            // coralSubsystem.setCoralPreset(currentCoralPreset);
            lockCoralArmPreset(selectedScoringPreset);
            if (!coralSubsystem.isHolding())
                isScoring = true;
        }).andThen(getGoToLockedPresetCommand().andThen(
                new InstantCommand(() -> {
                    operatorXbox.setRumble(RumbleType.kBothRumble, 1.0);
                }).andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> {
                    operatorXbox.setRumble(RumbleType.kBothRumble, 0.0);
                }))))).onlyIf(coralSubsystem.isHoldingSupplier()))
                .whileFalse(getStowCommand().alongWith(new InstantCommand(() -> {
                    isScoring = false;
                })));

        // wrist adjustment
        // Hold for now, until everything else is working
        // operatorXbox.rightStick().and(new BooleanSupplier() {
        // // deadzone
        // @Override
        // public boolean getAsBoolean() {
        // return Math.sqrt(Math.pow(operatorXbox.getRightX(), 2) +
        // Math.pow(operatorXbox.getRightY(), 2)) > 0.25;
        // }
        // }).whileTrue(new CoralWristFollowCommand(coralSubsystem, operatorXbox));

        // Score coral
        /*
         * .and(new BooleanSupplier() {
         * 
         * @Override
         * public boolean getAsBoolean() {
         * return coralSubsystem.isHolding() && isScoring;
         * }
         * })
         */
        operatorXbox.rightBumper().whileTrue(new ScoreCoralCommand(coralSubsystem));
        operatorXbox.rightBumper().whileFalse(getStowCommand());

        // Intake coral
        operatorXbox.rightTrigger().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !coralSubsystem.isHolding() && !isScoring;
            }
        }).whileTrue(new InstantCommand(() -> {
            System.out.println("Intaking");
            lockCoralArmPreset(CoralPresets.INTAKE);
        }).andThen(getGoToLockedPresetCommand()).andThen(new IntakeCoralCommand(coralSubsystem))
                .andThen(getStowCommand())).whileFalse(getStowCommand());

        // purge coral
        operatorXbox.button(7).whileTrue(new PurgeCoralIntakeCommand(coralSubsystem));

        coralAquisition.onChange(new InstantCommand(() -> {
            operatorXbox.setRumble(RumbleType.kBothRumble, 1.0);
        }).andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> {
            operatorXbox.setRumble(RumbleType.kBothRumble, 0.0);
        })));

        operatorXbox.leftBumper().onTrue(new InstantCommand(() -> {
            coralSubsystem.mirrorArm();
        }));

        /*
         * driver
         */
        // stop the climber
        driverXbox.x().onTrue(new InstantCommand(() -> {
            climberSubsystem.setOutput(0);
        }));
        // move the climber
        driverXbox.y().and(new BooleanSupplier() {
            private boolean deployed = true;

            @Override
            public boolean getAsBoolean() {
                deployed = !(deployed);
                return deployed;
            }
        }).onFalse(new InstantCommand(() -> {
            climberSubsystem.setOutput(1);
        })).onTrue(new InstantCommand(() -> {
            climberSubsystem.setOutput(-1);
        }));
        // TODO: something something maintainence
        driverXbox.button(6).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    System.out.print("swaaws");
                })));
        // set field orientation
        driverXbox.button(7).onTrue(new InstantCommand(() -> {
            swerveDriveSubsystem.setHeading(0);
        }));

        /*
         * coop
         */
        // algae floor / shoot
        driverXbox.leftBumper().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return algaeSubsystem.isHolding();
            }
        }).whileTrue(new ShootAlgaeCommand(algaeSubsystem));

        /////////////////// DEBUGGING //////////////////
        debugXboxController.a().onTrue(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetPitch(CoralPresets.LEVEL_4);
        })).onFalse(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetPitch(CoralPresets.STOW);
        }));

        debugXboxController.b().onTrue(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetRoll(CoralPresets.LEVEL_4);
        })).onFalse(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetRoll(CoralPresets.STOW);
        }));

        debugXboxController.x().onTrue(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetPivot(CoralPresets.LEVEL_4);
        })).onFalse(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetPivot(CoralPresets.STOW);
        }));
        debugXboxController.y().onTrue(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetElevator(CoralPresets.LEVEL_4);
        })).onFalse(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetElevator(CoralPresets.STOW);
        }));

        debugXboxController.rightBumper().whileTrue(new IntakeCoralCommand(coralSubsystem));
        debugXboxController.leftBumper().whileTrue(new ScoreCoralCommand(coralSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return new PathPlannerAuto("4-close-middle");

    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveDriveSubsystem;
    }

    public CommandXboxController getDriverXbox() {
        return driverXbox;
    }

    public CommandXboxController getOperatorXbox() {
        return operatorXbox;
    }
}
