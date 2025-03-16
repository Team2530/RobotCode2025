// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommand.DriveStyle;
import frc.robot.commands.algae.IntakeAlgaeCommand;
import frc.robot.commands.algae.PurgeAlgaeCommand;
import frc.robot.commands.algae.RemoveAlgaeCommand;
import frc.robot.commands.algae.ShootAlgaeBargeCommand;
import frc.robot.commands.algae.ShootAlgaeCommand;
import frc.robot.commands.coral.IntakeCoralCommand;
import frc.robot.commands.coral.PurgeCoralIntakeCommand;
import frc.robot.commands.coral.ScoreCoralCommand;
import frc.robot.commands.coral.motion.WristStowSafety;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LimelightType;
import frc.robot.subsystems.RobotMechanismLogger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import frc.robot.subsystems.coral.CoralSubsystem.MirrorPresets;
import frc.robot.util.LimelightContainer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {

    // private static final Limelight LL_BF = new Limelight(LimelightType.LL4,
    // "limelight-bf", true, true);
    private static final Limelight LL_BR = new Limelight(LimelightType.LL4, "limelight-br", true, true);
    private static final Limelight LL_BL = new Limelight(LimelightType.LL4, "limelight-bl", true, true);
    private static final Limelight LL_FR = new Limelight(LimelightType.LL4, "limelight-fr", true, true);

    @Logged
    public static final LimelightContainer LLContainer = new LimelightContainer(LL_BR, LL_BL, LL_FR);

    // @Logged
    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    // @Logged
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);
    // private final CommandXboxController debugXboxController = new
    // CommandXboxController(3);

    // private final CommandXboxController debugXbox = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;

    @Logged
    public final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();

    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();
    @Logged
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    @Logged
    private final CoralSubsystem coralSubsystem = new CoralSubsystem(swerveDriveSubsystem);

    // NOTE: Removed to prevent loop overruns while the robot does not have the
    // algae manipulator installed.
    @Logged
    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

    @Logged
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(operatorXbox.getHID());

    private final RobotMechanismLogger robotLogger = new RobotMechanismLogger(coralSubsystem, swerveDriveSubsystem,
            algaeSubsystem);

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        DataLogManager.logNetworkTables(true);
        DataLogManager.start();

        swerveDriveSubsystem.setDefaultCommand(normalDrive);

        NamedCommands.registerCommand("L1",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.LEVEL_1);
                }).andThen(coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)));

        NamedCommands.registerCommand("L2",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.LEVEL_2);
                }).andThen(coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)));

        NamedCommands.registerCommand("L3",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.LEVEL_3);
                }).andThen(coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)));

        NamedCommands.registerCommand("L4",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.LEVEL_4);
                }).andThen(coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)));

        NamedCommands.registerCommand("Score",
                new WaitCommand(Constants.AutoConstants.SCORE_WAIT_BEFORE_SECONDS).andThen(new ScoreCoralCommand(
                        coralSubsystem).withTimeout(Constants.AutoConstants.SCORE_WAIT_AFTER_SECONDS)));

        NamedCommands.registerCommand("Intake",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(coralSubsystem.getGoToLockedPresetFASTCommand(algaeSubsystem,
                                currentLockedPresetSupplier))
                        .andThen(new IntakeCoralCommand(coralSubsystem))
                        .andThen(getStowCommand()));

        NamedCommands.registerCommand("Start Intake",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(coralSubsystem.getGoToLockedPresetFASTCommand(algaeSubsystem,
                                currentLockedPresetSupplier))
                        .andThen(new InstantCommand(() -> {
                            CommandScheduler.getInstance().schedule(new IntakeCoralCommand(coralSubsystem));
                        })));

        NamedCommands.registerCommand("Start Intake L",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(coralSubsystem.getGoToLockedPresetSideFASTCommand(algaeSubsystem,
                                currentLockedPresetSupplier,
                                MirrorPresets.LEFT))
                        .andThen(new InstantCommand(() -> {
                            CommandScheduler.getInstance().schedule(new IntakeCoralCommand(coralSubsystem));
                        })));

        NamedCommands.registerCommand("Start Intake R",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(coralSubsystem.getGoToLockedPresetSideFASTCommand(algaeSubsystem,
                                currentLockedPresetSupplier, MirrorPresets.RIGHT))
                        .andThen(new InstantCommand(() -> {
                            CommandScheduler.getInstance().schedule(new IntakeCoralCommand(coralSubsystem));
                        })));

        NamedCommands.registerCommand("Wait Intake",
                new WaitUntilCommand(coralSubsystem.isHoldingSupplier()).andThen(new InstantCommand(() -> {
                    CommandScheduler.getInstance().schedule(getStowCommand());
                })));

        NamedCommands.registerCommand("Stow", getStowCommand());

        NamedCommands.registerCommand("Algae Low",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.ALGAE_REM_LOW);
                })
                        .andThen(
                                new ParallelCommandGroup(
                                        new RemoveAlgaeCommand(algaeSubsystem),
                                        coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem,
                                                currentLockedPresetSupplier))));

        NamedCommands.registerCommand("Algae High",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.ALGAE_REM_HIGH);
                })
                        .andThen(
                                new ParallelCommandGroup(
                                        new RemoveAlgaeCommand(algaeSubsystem),
                                        coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem,
                                                currentLockedPresetSupplier))));

        NamedCommands.registerCommand("Grab Low", new InstantCommand(() -> {
            lockCoralArmPreset(CoralPresets.ALGAE_ACQUIRE_LOW);
        }).andThen((coralSubsystem
                .getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)
                .alongWith(new IntakeAlgaeCommand(algaeSubsystem)))
                .onlyIf(algaeSubsystem.getIntake().getNotHoldingSupplier())));

        NamedCommands.registerCommand("Grab High", new InstantCommand(() -> {
            lockCoralArmPreset(CoralPresets.ALGAE_ACQUIRE_HIGH);
        }).andThen((coralSubsystem
                .getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)
                .alongWith(new IntakeAlgaeCommand(algaeSubsystem)))
                .onlyIf(algaeSubsystem.getIntake().getNotHoldingSupplier())));

        swerveDriveSubsystem.configurePathplanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private CoralPresets selectedScoringPreset = CoralPresets.STOW;
    private int selectedLevel = 0;
    private CoralPresets lockedPreset = CoralPresets.STOW;
    private boolean isScoring = false;

    BooleanSupplier coralSafe = new BooleanSupplier() {
        public boolean getAsBoolean() {
            return !algaeSubsystem.isHolding();
        };
    };

    BooleanSupplier algaeGrabSafe = new BooleanSupplier() {
        public boolean getAsBoolean() {
            return !coralSubsystem.isHolding();
        };
    };

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

    private Command getStowCommand() {
        return new InstantCommand(() -> {
            CoralPresets preset = CoralPresets.STOW;
            // If holding algae, use the corresponding algae stow preset
            if (algaeSubsystem.isHolding()) {
                if (lockedPreset == CoralPresets.ALGAE_ACQUIRE_HIGH || lockedPreset == CoralPresets.ALGAE_STOW_HIGH) {
                    preset = CoralPresets.ALGAE_STOW_HIGH;
                } else {
                    preset = CoralPresets.ALGAE_STOW_LOW;
                }
            }
            lockCoralArmPreset(preset);
            algaeSubsystem.setAlgaePreset(algaeSubsystem.isHolding() ? AlgaePresets.HOLD : AlgaePresets.STOW);
        }).andThen(new ConditionalCommand(
                coralSubsystem.getGoToLockedPresetAlgaeSafeCommand(algaeSubsystem, currentLockedPresetSupplier),
                new WristStowSafety(coralSubsystem)
                        .andThen(coralSubsystem.getGoToLockedPresetFASTCommand(algaeSubsystem,
                                currentLockedPresetSupplier)),
                algaeSubsystem.getIntake().getHoldingSupplier()));
    }

    private Command getGoToCoralScoringPositionCommand() {
        return new InstantCommand(() -> {
            lockCoralArmPreset(selectedScoringPreset);
            isScoring = true;
            SmartDashboard.putString("Operator Control", "Going to Coral Scoring Preset: " + lockedPreset.toString());
        }).andThen(
                coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)
                        .andThen(new InstantCommand(() -> {
                            operatorXbox.setRumble(RumbleType.kBothRumble, 1.0);
                        }).andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> {
                            operatorXbox.setRumble(RumbleType.kBothRumble, 0.0);
                        }))));
    }

    private Command getGoToAlgaeScoringPositionCommand() {
        return new InstantCommand(() -> {
            lockCoralArmPreset(selectedLevel == 1 ? CoralPresets.ALGAE_PROCESSOR : CoralPresets.ALGAE_BARGE);
            SmartDashboard.putString("Operator Control", "Going to Algae Scoring Preset: " + lockedPreset.toString());
        }).andThen(coralSubsystem.getGoToLockedPresetAlgaeSafeCommand(algaeSubsystem, currentLockedPresetSupplier));
    }

    private Command getScoreAlgaeCommand() {
        return new ConditionalCommand(new ShootAlgaeBargeCommand(algaeSubsystem), new ShootAlgaeCommand(algaeSubsystem),
                new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return currentLockedPresetSupplier.get() == CoralPresets.ALGAE_BARGE;
                    }
                });
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

        // Driver assist controls
        driverXbox.leftTrigger().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return driverXbox.getLeftTriggerAxis() > 0.05;
            }
        }).onTrue(new ConditionalCommand(new InstantCommand(() -> {
            normalDrive.setDriveStyle(DriveStyle.REEF_ASSIST);
        }), new InstantCommand(() -> {
            normalDrive.setDriveStyle(DriveStyle.INTAKE_ASSIST);
        }), coralSubsystem.isHoldingSupplier())).onFalse(new InstantCommand(() -> {
            normalDrive.setDriveStyle(DriveStyle.FIELD_ORIENTED);
        }));

        // L1
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_1;
            selectedLevel = 1;
        }));
        // L2
        operatorXbox.x().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_2;
            selectedLevel = 2;
        }));
        // L3
        operatorXbox.y().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_3;
            selectedLevel = 3;
        }));
        // L4
        operatorXbox.b().onTrue(new InstantCommand(() -> {
            selectedScoringPreset = CoralPresets.LEVEL_4;
            selectedLevel = 4;
        }));

        // Move arm to coral scoring preset
        operatorXbox.rightTrigger().and(coralSafe)
                .whileTrue(getGoToCoralScoringPositionCommand().onlyIf(coralSubsystem.isHoldingSupplier()));
        // Stow arm
        operatorXbox.rightTrigger().whileFalse(getStowCommand().alongWith(new InstantCommand(() -> {
            isScoring = false;
        })));

        // Driver scoring:
        // - If holding coral, score coral
        // - If not holding coral:
        // -- If holding algae, score algae
        // -- If not holding algae, spin coral intake
        driverXbox.rightBumper().and(coralSubsystem.getIntake().getHoldingSupplier())
                .whileTrue(new ScoreCoralCommand(coralSubsystem));
        driverXbox.rightBumper().and(coralSubsystem.getIntake()
                .getNotHoldingSupplier())
                .whileTrue(new ConditionalCommand(getScoreAlgaeCommand(),
                        new ScoreCoralCommand(coralSubsystem), algaeSubsystem.getIntake().getHoldingSupplier()));

        // Operator tap-to-stow
        operatorXbox.rightBumper().whileFalse(getStowCommand());

        // Intake coral
        operatorXbox.rightTrigger().and(coralSafe).and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !coralSubsystem.isHolding() && !isScoring;
            }
        }).whileTrue(new InstantCommand(() -> {
            SmartDashboard.putString("Operator Control", "Intaking Coral");
            lockCoralArmPreset(CoralPresets.INTAKE);
        }).andThen(coralSubsystem.getGoToLockedPresetFASTCommand(algaeSubsystem,
                currentLockedPresetSupplier)).andThen(new IntakeCoralCommand(coralSubsystem))
                .andThen(getStowCommand()))
                .whileFalse(new ConditionalCommand(getStowCommand(), new InstantCommand(),
                        coralSubsystem.isHoldingSupplier()));

        // Purge gamepieces
        operatorXbox.button(7).whileTrue(new ParallelCommandGroup(new PurgeCoralIntakeCommand(coralSubsystem),
                new PurgeAlgaeCommand(algaeSubsystem)));

        // Reset climber deploy (allow to pull back in)
        operatorXbox.button(8).onTrue(new InstantCommand(() -> {
            climberSubsystem.resetClimberDeploy();
        }));

        // Rumble on coral acquisition
        coralAquisition.onChange(new InstantCommand(() -> {
            operatorXbox.setRumble(RumbleType.kBothRumble, 1.0);
            driverXbox.setRumble(RumbleType.kBothRumble, 1.0);
        }).andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> {
            operatorXbox.setRumble(RumbleType.kBothRumble, 0.0);
            driverXbox.setRumble(RumbleType.kBothRumble, 0.0);
        })));

        // Algae removal
        operatorXbox.leftBumper().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return selectedLevel == 2 || selectedLevel == 3;
            }
        }).whileTrue(
                new InstantCommand(() -> {
                    lockCoralArmPreset(selectedLevel == 2 ? CoralPresets.ALGAE_REM_LOW : CoralPresets.ALGAE_REM_HIGH);
                }).andThen(
                        new ParallelCommandGroup(
                                new RemoveAlgaeCommand(algaeSubsystem),
                                coralSubsystem.getGoToLockedPresetCommandV2(algaeSubsystem,
                                        currentLockedPresetSupplier))));
        // Stow
        operatorXbox.leftBumper().onFalse(getStowCommand());

        // Algae intaking
        operatorXbox.leftTrigger().and(algaeGrabSafe)
                .and(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return selectedLevel == 2 || selectedLevel == 3;
                    }
                }).whileTrue(
                        new InstantCommand(() -> {
                            lockCoralArmPreset(
                                    selectedLevel == 2 ? CoralPresets.ALGAE_ACQUIRE_LOW
                                            : CoralPresets.ALGAE_ACQUIRE_HIGH);
                        }).andThen((coralSubsystem
                                .getGoToLockedPresetCommandV2(algaeSubsystem, currentLockedPresetSupplier)
                                .alongWith(new IntakeAlgaeCommand(algaeSubsystem)))
                                .onlyIf(algaeSubsystem.getIntake().getNotHoldingSupplier())));
        // Stow
        operatorXbox.leftTrigger().onFalse(getStowCommand());

        // Algae scoring
        operatorXbox.leftTrigger().and(algaeSubsystem.getIntake().getHoldingSupplier()).and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return selectedLevel == 1 || selectedLevel == 4;
            }
        }).whileTrue(getGoToAlgaeScoringPositionCommand());
        operatorXbox.leftTrigger().whileFalse(getStowCommand());

        // Driver elevator zeroing
        driverXbox.button(7).onTrue(new InstantCommand(() -> {
            // Move elevator down to zero
            coralSubsystem.getElevator().startZeroElevator();
            // Re-zero wrist relative encoder!!!
            coralSubsystem.getCoralArm().reset();
        })).onFalse(new InstantCommand(() -> {
            coralSubsystem.getElevator().endZeroElevator();
        }));

        /*
         * coop
         */
        // algae floor / shoot
        // driverXbox.leftBumper().and(new BooleanSupplier() {
        // @Override
        // public boolean getAsBoolean() {
        // return algaeSubsystem.isHolding();
        // }
        // }).whileTrue(new ShootAlgaeCommand(algaeSubsystem));

        /////////////////// DEBUGGING //////////////////
        // debugXboxController.a().onTrue(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetPitch(CoralPresets.LEVEL_4);
        // })).onFalse(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetPitch(CoralPresets.STOW);
        // }));

        // debugXboxController.b().onTrue(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetRoll(CoralPresets.LEVEL_4);
        // })).onFalse(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetRoll(CoralPresets.STOW);
        // }));

        // debugXboxController.x().onTrue(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetPivot(CoralPresets.LEVEL_4);
        // })).onFalse(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetPivot(CoralPresets.STOW);
        // }));
        // debugXboxController.y().onTrue(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetElevator(CoralPresets.LEVEL_4);
        // })).onFalse(new InstantCommand(() -> {
        // coralSubsystem.setCoralPresetElevator(CoralPresets.STOW);
        // }));

        // debugXboxController.rightBumper().whileTrue(new
        // IntakeCoralCommand(coralSubsystem));
        // debugXboxController.leftBumper().whileTrue(new
        // ScoreCoralCommand(coralSubsystem));

        // // Algae debugging!!!
        // debugXboxController.povUp().onTrue(new InstantCommand(() -> {
        // algaeSubsystem.setAlgaePreset(AlgaePresets.REMOVE);
        // }));

        // debugXboxController.povDown().onTrue(new InstantCommand(() -> {
        // algaeSubsystem.setAlgaePreset(AlgaePresets.STOW);
        // }));

        // debugXboxController.povLeft().whileTrue(new
        // RemoveAlgaeCommand(algaeSubsystem));
        // debugXboxController.povRight().whileTrue(new
        // IntakeAlgaeCommand(algaeSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        swerveDriveSubsystem.setGyroToEstimate();
        if (Robot.isSimulation()) {
            coralSubsystem.simSetHolding(true);
        }
        return autoChooser.getSelected();
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
