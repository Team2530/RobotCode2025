package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualClimberCommand extends Command {
    private final XboxController operator;
    ClimberSubsystem climber;

    public ManualClimberCommand(ClimberSubsystem climber, XboxController xbox) {
        this.climber = climber;
        this.operator = xbox;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (operator.getPOV(0) == 1) {
            this.climber.setOutput(0.25);
        } else if (operator.getPOV(180) == 1) {
            this.climber.setOutput(-0.25);
        } else {
            this.climber.setOutput(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    public boolean isFinished() {
        return false;
    }
}
