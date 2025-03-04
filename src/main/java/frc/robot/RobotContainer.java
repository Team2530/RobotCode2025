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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.algae.ShootAlgaeCommand;
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
import frc.robot.subsystems.RobotMechanismLogger;
import frc.robot.subsystems.Limelight.LimelightType;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;
import frc.robot.util.LimelightAssistance;
import frc.robot.util.LimelightContainer;

import frc.robot.util.Reef;
import frc.robot.util.Reef.ReefBranch;

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

    @Logged
    public static final LimelightContainer LLContainer = new LimelightContainer(LL_BF, LL_BR, LL_BL);

    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController debugXboxController = new CommandXboxController(3);

    // private final CommandXboxController debugXbox = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;

    @Logged
    public final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    @Logged
    public final LimelightAssistance limelightAssistance = new LimelightAssistance(swerveDriveSubsystem);
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    @Logged
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    @Logged
    private final CoralSubsystem coralSubsystem = new CoralSubsystem(limelightAssistance, swerveDriveSubsystem);

    // NOTE: Removed to prevent loop overruns while the robot does not have the
    // algae manipulator installed.
    // @Logged
    // private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

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
                new ScoreCoralCommand(coralSubsystem).withTimeout(Constants.AutoConstants.SCORE_WAIT_SECONDS));

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
                        .andThen(new IntakeCoralCommand(coralSubsystem)));

        NamedCommands.registerCommand("Wait Intake",
                new WaitUntilCommand(coralSubsystem.isHoldingSupplier()).andThen(getStowCommand()));

        NamedCommands.registerCommand("Stow", getStowCommand());

        swerveDriveSubsystem.configurePathplanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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

    private Command getGoToLockedPresetCommandV2() {
        return new InstantCommand(() -> {
            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE) {
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

    // Goes to a preset more quickly by moving pitch+pivot+roll at the same time,
    // but can throw coral. Good for intaking
    private Command getGoToLockedPresetFASTCommand() {
        return new InstantCommand(() -> {
            if (currentLockedPresetSupplier.get() == CoralPresets.INTAKE) {
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
            lockCoralArmPreset(CoralPresets.STOW);
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
        driverXbox.rightBumper().whileTrue(new ScoreCoralCommand(coralSubsystem));
        operatorXbox.rightBumper().whileFalse(getStowCommand());
        driverXbox.rightBumper().whileFalse(getStowCommand());

        // Intake coral
        operatorXbox.rightTrigger().and(new BooleanSupplier() {
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
        // driverXbox.leftBumper().and(new BooleanSupplier() {
        // @Override
        // public boolean getAsBoolean() {
        // return algaeSubsystem.isHolding();
        // }
        // }).whileTrue(new ShootAlgaeCommand(algaeSubsystem));

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
