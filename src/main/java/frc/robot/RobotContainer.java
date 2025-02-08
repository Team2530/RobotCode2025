// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.algae.ShootAlgaeCommand;
import frc.robot.commands.coral.CoralWristFollowCommand;
import frc.robot.commands.coral.IntakeCoralCommand;
import frc.robot.commands.coral.PurgeCoralIntakeCommand;
import frc.robot.commands.coral.ScoreCoralCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.*;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaePresets;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralPresets;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorCommand.ElevatorPresets;
import frc.robot.commands.ElevatorFollowCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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

    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    // private final CommandXboxController debugXbox = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;

    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    // private final LimeLightSubsystem limeLightSubsystem = new
    // LimeLightSubsystem();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private CoralPresets currentCoralPreset = CoralPresets.STOW;

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
        NamedCommands.registerCommand("Testing", elevatorToTop);
    }

    // Command shootAction =
    // Command alignAction = ; // Self-deadlines
    // Command spoolAction =
    // Command intakeAction = ;
    
    private ElevatorCommand elevatorToTop = new ElevatorCommand(elevator, ElevatorPresets.TOP, 0.0);
    private ElevatorCommand elevatorToMiddle = new ElevatorCommand(elevator, ElevatorPresets.MIDDLE, 0.0);
    private ElevatorCommand elevatorToStow = new ElevatorCommand(elevator, ElevatorPresets.STOW, 0.0);


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
        // low algae
        operatorXbox.leftBumper().onTrue(new InstantCommand(() -> {
            algaeSubsystem.setAlgaePreset(AlgaePresets.FLOOR);
        }));
        // high algae
        operatorXbox.rightBumper().onTrue(new InstantCommand(() -> {
            algaeSubsystem.setAlgaePreset(AlgaePresets.STOW);
        }));

        // L1 
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            currentCoralPreset = CoralPresets.LEVEL_1;
        }));
        // L2
        operatorXbox.x().onTrue(new InstantCommand(() -> {
            currentCoralPreset = CoralPresets.LEVEL_2;
        }));
        // L3
        operatorXbox.y().onTrue(new InstantCommand(() -> {
            currentCoralPreset = CoralPresets.LEVEL_3;
        }));
        // L4
        operatorXbox.b().onTrue(new InstantCommand(() -> {
            currentCoralPreset = CoralPresets.LEVEL_4;
        }));
        // go to preset position
        operatorXbox.rightTrigger().onTrue(new InstantCommand(() -> {
            coralSubsystem.setCoralPreset(currentCoralPreset);
        }));
        // wrist adjustment
        operatorXbox.rightStick().and(new BooleanSupplier() {
            // deadzone
            @Override
            public boolean getAsBoolean() {
                return Math.sqrt(Math.pow(operatorXbox.getRightX(),2) + Math.pow(operatorXbox.getRightY(),2)) > 0.25; 
            }
        }).whileTrue(new CoralWristFollowCommand(coralSubsystem, operatorXbox));
        // score / intake coral
        operatorXbox.rightBumper().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return coralSubsystem.isHolding();
            }
        }).whileTrue(new ScoreCoralCommand(coralSubsystem));
        operatorXbox.rightBumper().and(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !coralSubsystem.isHolding();
            }
        }).whileTrue(new IntakeCoralCommand(coralSubsystem));
        // purge coral
        operatorXbox.button(7).whileTrue(new PurgeCoralIntakeCommand(coralSubsystem));
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
            })
        ));
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
