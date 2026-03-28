package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Constants.IndexerConstants;

/**
 * Subsystem that controls the indexer motor used to feed game pieces into the shooter.
 */
public class IndexerSubsystem extends SubsystemBase {
    // Indexer motor.
    private final SparkMax indexerMotor;

    // Motor configuration for the intake motor.
    public static final SparkMaxConfig indexerConfig = new SparkMaxConfig();

    static {
        indexerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }
    
    /**
     * Constructor for the IndexerSubsystem. Initializes the indexer motor and applies the configuration.
     */
    @SuppressWarnings("removal") // Suppress warnings about deprecated ResetMode and PersistMode usage in SparkMax configuration.
    public IndexerSubsystem() {
        // Initialize the indexer motor using configured CAN ID.
        indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

        // Configure the indexer motor using safe parameter reset and persistent parameter storage.
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Constructs a delayed command that starts the indexer and allows the shooter to spin up before
     * feeding, while ensuring that the indexer is stopped when the command ends or is interrupted.
     * 
     * @return command that leaves the indexer stopped and forces zero output on end/interruption
     */
    public Command runIndexerCommand() {
        return Commands.sequence(
            Commands.waitSeconds(2.0), // Delays the start of the indexer by a set time interval
            run(() -> setIndexerSpeed(1.0))
        ).finallyDo(interrupted -> setIndexerSpeed(0.0)); // Ensure indexer is stopped when this command ends or is interrupted.
    }

    /**
     * Sets the speed of the indexer motor.
     * 
     * @param speed The speed to set the motor to (between -1.0 and 1.0).
     */
    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }
}
