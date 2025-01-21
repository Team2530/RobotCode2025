package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class NewElevatorSubsystem extends SubsystemBase {

    private final SparkMax leaderMotor = new SparkMax(Elevator.elevatorOnePort, MotorType.kBrushless);
    private final SparkMax followerMotor = new SparkMax(Elevator.elevatorTwoPort, MotorType.kBrushless);

    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    private final ElevatorFeedforward feedForward = Elevator.feedForward;

    public NewElevatorSubsystem() {
        
    }
}
