// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.ElevatorCommand.ElevatorPresets;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystem.RotationStyle;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.*;

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

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        DataLogManager.logNetworkTables(true);
        DataLogManager.start();

        // NamedCommands.registerCommand("NoNote", new (
        // new WaitForCommand(1.0),
        // new WaitUntilCommand(new BooleanSupplier() {
        // @Override
        // public boolean getAsBoolean() {
        // return !intake.containsNote().getAsBoolean();
        // }
        // })
        // ));

        /*
         * NamedCommands.registerCommand("Shoot Close", new SequentialCommandGroup(
         * new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
         * new WaitCommand(2),
         * new AlignNoteCommand(intake, shooter),
         * new PrepNoteCommand(shooter, intake),
         * new PrepShooterCommand(intake, shooter, 0.8),
         * new ShootCommand(shooter, intake)
         * ));
         * NamedCommands.registerCommand("Pickup", new SequentialCommandGroup(
         * new InstantCommand(() -> {arm.setArmPreset(Presets.INTAKE);}),
         * new IntakeCommand(intake)));
         */
        // Test Auto Week 0
        // return new SequentialCommandGroup(
        // new InstantCommand(() -> {arm.setArmPreset(Presets.SHOOT_HIGH);}),
        // new WaitCommand(2),
        // new AlignNoteCommand(intake, shooter),
        // new PrepNoteCommand(shooter, intake),
        // new PrepShooterCommand(intake, shooter, 0.8),
        // new InstantCommand(() -> System.out.println("HELLLLLOOO")),
        // new ShootCommand(shooter, intake)
        // );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
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
        operatorXbox.a()
                .onTrue(elevatorToStow);
        operatorXbox.x()
                .onTrue(elevatorToMiddle);
        operatorXbox.y()
                .onTrue(elevatorToTop);

        operatorXbox.b().whileTrue(new ElevatorFollowCommand(elevator, new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return (operatorXbox.getLeftY() * -0.5 + 0.5)
                        * Constants.Elevator.PhysicalParameters.elevatorHeightMeters;
            }
        }));

        operatorXbox.povUp().debounce(0.02).onTrue(new InstantCommand(() -> {
            elevator.setPosition(elevator.getGoalPosition() + 0.1);
        }));

        operatorXbox.povDown().debounce(0.02).onTrue(new InstantCommand(() -> {
            elevator.setPosition(elevator.getGoalPosition() - 0.1);
        }));
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
