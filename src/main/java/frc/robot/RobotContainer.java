// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommand.DriveStyle;
import frc.robot.commands.algae.IntakeAlgaeCommand;
import frc.robot.commands.algae.RemoveAlgaeCommand;
import frc.robot.commands.coral.IntakeCoralCommand;
import frc.robot.commands.coral.PurgeCoralIntakeCommand;
import frc.robot.commands.coral.ScoreCoralCommand;
import frc.robot.commands.coral.motion.MoveElevator;
import frc.robot.commands.coral.motion.MovePitch;
import frc.robot.commands.coral.motion.MovePivot;
import frc.robot.commands.coral.motion.MoveRoll;
import frc.robot.commands.coral.motion.StowArm;
import frc.robot.commands.coral.motion.WaitArmClearance;
import frc.robot.commands.coral.motion.WaitElevatorApproach;
import frc.robot.commands.coral.motion.WaitRollFinished;
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
import frc.robot.util.LimelightAssistance;
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

    private static final Limelight LL_BF = new Limelight(LimelightType.LL4, "limelight-bf", true, true);
    private static final Limelight LL_BR = new Limelight(LimelightType.LL4, "limelight-br", true, true);
    private static final Limelight LL_BL = new Limelight(LimelightType.LL4, "limelight-bl", true, true);
    private static final Limelight LL_FR = new Limelight(LimelightType.LL4, "limelight-fr", true, true);

    @Logged
    public static final LimelightContainer LLContainer = new LimelightContainer(LL_BF, LL_BR, LL_BL, LL_FR);

    // @Logged
    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    // @Logged
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController debugXboxController = new CommandXboxController(3);

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

    private final RobotMechanismLogger robotLogger = new RobotMechanismLogger(coralSubsystem);

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
                }).andThen(getGoToLockedPresetCommandV2()));

        NamedCommands.registerCommand("L2",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.LEVEL_2);
                }).andThen(getGoToLockedPresetCommandV2()));

        NamedCommands.registerCommand("L3",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.LEVEL_3);
                }).andThen(getGoToLockedPresetCommandV2()));

        NamedCommands.registerCommand("L4",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.LEVEL_4);
                }).andThen(getGoToLockedPresetCommandV2()));

        NamedCommands.registerCommand("Score",
                new WaitCommand(Constants.AutoConstants.SCORE_WAIT_BEFORE_SECONDS).andThen(new ScoreCoralCommand(
                        coralSubsystem).withTimeout(Constants.AutoConstants.SCORE_WAIT_AFTER_SECONDS)));

        NamedCommands.registerCommand("Intake",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(getGoToLockedPresetFASTCommand())
                        .andThen(new IntakeCoralCommand(coralSubsystem))
                        .andThen(getStowCommand()));

        NamedCommands.registerCommand("Start Intake",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(getGoToLockedPresetFASTCommand())
                        .andThen(new InstantCommand(() -> {
                            CommandScheduler.getInstance().schedule(new IntakeCoralCommand(coralSubsystem));
                        })));

        NamedCommands.registerCommand("Start Intake L",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(getGoToLockedPresetSideFASTCommand(
                                MirrorPresets.LEFT))
                        .andThen(new InstantCommand(() -> {
                            CommandScheduler.getInstance().schedule(new IntakeCoralCommand(coralSubsystem));
                        })));

        NamedCommands.registerCommand("Start Intake R",
                new InstantCommand(() -> {
                    lockCoralArmPreset(CoralPresets.INTAKE);
                })
                        .andThen(getGoToLockedPresetSideFASTCommand(MirrorPresets.RIGHT))
                        .andThen(new InstantCommand(() -> {
                            CommandScheduler.getInstance().schedule(new IntakeCoralCommand(coralSubsystem));
                        })));

        NamedCommands.registerCommand("Wait Intake",
                new WaitUntilCommand(coralSubsystem.isHoldingSupplier()).andThen(getStowCommand()));

        NamedCommands.registerCommand("Stow", getStowCommand());

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

    private Command getGoToLockedPresetCommandV2() {
        return new InstantCommand(() -> {
            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE) {
                algaeSubsystem.setAlgaePreset(AlgaePresets.OUT_OF_THE_WAY);
                coralSubsystem.autoSetMirrorIntake();
            } else {
                coralSubsystem.autoSetMirrorScoring();
            }

            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        }).andThen(new StowArm(coralSubsystem))
                .andThen(new ParallelCommandGroup(
                        new MoveElevator(coralSubsystem, currentLockedPresetSupplier),
                        new MovePivot(coralSubsystem, currentLockedPresetSupplier),
                        new WaitArmClearance(coralSubsystem)
                                .andThen(new MoveRoll(coralSubsystem, currentLockedPresetSupplier)),
                        new WaitRollFinished(coralSubsystem).andThen(new WaitElevatorApproach(coralSubsystem, 0.5))
                                .andThen(new MovePitch(coralSubsystem, currentLockedPresetSupplier))))
                .andThen(new InstantCommand(() -> {
                    SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString() + " - Done");
                }));
    }

    private Command getGoToLockedPresetSideFASTCommand(MirrorPresets mirrorSide) {
        return new InstantCommand(() -> {
            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE)
                algaeSubsystem.setAlgaePreset(AlgaePresets.OUT_OF_THE_WAY);
            coralSubsystem.mirrorArm(mirrorSide);

            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        }).andThen(new StowArm(coralSubsystem))
                .andThen(new MoveElevator(coralSubsystem, currentLockedPresetSupplier))
                .andThen(new ParallelCommandGroup(
                        new MovePivot(coralSubsystem, currentLockedPresetSupplier),
                        new MoveRoll(coralSubsystem, currentLockedPresetSupplier),
                        new MovePitch(coralSubsystem, currentLockedPresetSupplier)))
                .andThen(new InstantCommand(() -> {
                    SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString() + " - Done");
                }));
    }

    // Goes to a preset more quickly by moving pitch+pivot+roll at the same time,
    // but can throw coral. Good for intaking
    private Command getGoToLockedPresetFASTCommand() {
        return new InstantCommand(() -> {
            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE) {
                algaeSubsystem.setAlgaePreset(AlgaePresets.OUT_OF_THE_WAY);

                coralSubsystem.autoSetMirrorIntake();
            } else {
                coralSubsystem.autoSetMirrorScoring();
            }
            SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString());
        }).andThen(new StowArm(coralSubsystem))
                .andThen(new MoveElevator(coralSubsystem, currentLockedPresetSupplier))
                .andThen(new ParallelCommandGroup(
                        new MovePivot(coralSubsystem, currentLockedPresetSupplier),
                        new MoveRoll(coralSubsystem, currentLockedPresetSupplier),
                        new MovePitch(coralSubsystem, currentLockedPresetSupplier)))
                .andThen(new InstantCommand(() -> {
                    SmartDashboard.putString("Going to", currentLockedPresetSupplier.get().toString() + " - Done");
                }));
    }

    private Command getStowCommand() {
        return new InstantCommand(() -> {
            lockCoralArmPreset(
                    algaeSubsystem.isHolding()
                            ? ((lockedPreset == CoralPresets.ALGAE_ACQUIRE_HIGH
                                    || lockedPreset == CoralPresets.ALGAE_STOW_HIGH) ? CoralPresets.ALGAE_STOW_HIGH
                                            : CoralPresets.ALGAE_STOW_LOW)
                            : CoralPresets.STOW);
            // lockCoralArmPreset(CoralPresets.STOW);
            algaeSubsystem.setAlgaePreset(AlgaePresets.STOW);
        }).andThen(getGoToLockedPresetFASTCommand());
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

        // Score!!!
        operatorXbox.rightTrigger().and(coralSafe).whileTrue((new InstantCommand(() -> {
            // coralSubsystem.setCoralPreset(currentCoralPreset);
            lockCoralArmPreset(selectedScoringPreset);
            if (!coralSubsystem.isHolding())
                isScoring = true;
        }).andThen(getGoToLockedPresetCommandV2().andThen(
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
        driverXbox.rightBumper().and(coralSafe).whileTrue(new ScoreCoralCommand(coralSubsystem));
        operatorXbox.rightBumper().and(coralSafe).whileFalse(getStowCommand());
        driverXbox.rightBumper().and(coralSafe).whileFalse(getStowCommand());

        // Intake coral
        operatorXbox.rightTrigger().and(coralSafe).and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !coralSubsystem.isHolding() && !isScoring;
            }
        }).whileTrue(new InstantCommand(() -> {
            System.out.println("Intaking");
            lockCoralArmPreset(CoralPresets.INTAKE);
        }).andThen(getGoToLockedPresetFASTCommand()).andThen(new IntakeCoralCommand(coralSubsystem))
                .andThen(getStowCommand())).whileFalse(getStowCommand());

        // purge coral
        operatorXbox.button(7).whileTrue(new PurgeCoralIntakeCommand(coralSubsystem));
        operatorXbox.button(8).onTrue(new InstantCommand(() -> {
            climberSubsystem.resetClimberDeploy();
        }));

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
                                getGoToLockedPresetCommandV2())))
                .onFalse(getStowCommand());

        // Algae intaking
        // operatorXbox.leftTrigger().and(algaeSubsystem.getIntake().getNotHoldingSupplier()).and(new
        // BooleanSupplier() {
        // @Override
        // public boolean getAsBoolean() {
        // return selectedLevel == 2 || selectedLevel == 3;
        // }
        // }).whileTrue(
        // new InstantCommand(() -> {
        // lockCoralArmPreset(
        // selectedLevel == 2 ? CoralPresets.ALGAE_ACQUIRE_LOW :
        // CoralPresets.ALGAE_ACQUIRE_HIGH);
        // }).andThen(getGoToLockedPresetCommandV2())
        // .andThen(new IntakeAlgaeCommand(algaeSubsystem)));
        // .onFalse(getStowCommand());

        /*
         * driver
         */
        // stop the climber

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

        // TODO: Fix zeroing!!!!!
        // driverXbox.button(7).whileTrue(new RepeatCommand(new InstantCommand(() -> {
        // coralSubsystem.getElevator().zeroElevator();
        // })));

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
        debugXboxController.y().onTrue(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetElevator(CoralPresets.LEVEL_4);
        })).onFalse(new InstantCommand(() -> {
            coralSubsystem.setCoralPresetElevator(CoralPresets.STOW);
        }));

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
