package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Constants.IntakeConstants;

/**
 * Subsystem that controls the intake motor used to collect game pieces off the floor.
 */
public class IntakeSubsystem extends SubsystemBase {
    // Intake motor.
    private final SparkMax intakeMotor;

    // Motor configuration for the intake motor.
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }

    /**
     * Constructor for the IntakeSubsystem. Initializes the intake motor and applies the configuration.
     */
    @SuppressWarnings("removal") // Suppress warnings about deprecated ResetMode and PersistMode usage in SparkMax configuration.
    public IntakeSubsystem() {
        // Initialize the intake motor using configured CAN ID.
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Configure the intake motor using safe parameter reset and persistent parameter storage.
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Creates a command that continuously runs the intake motor at a fixed speed.
     * On interruption (when the bound button is toggled), stop the motor.
     * 
     * @return a command that runs the intake at a set speed and stops on interruption.
     */
    public Command continuousIntakeCommand() { //
        return runEnd(
            () -> setIntakeSpeed(-0.9), // Run intake at a set speed.
            () -> setIntakeSpeed(0.0) // Stop intake when command is interupted.
        );
    }

    /**
     * Sets the speed of the intake motor.
     * 
     * @param speed The speed to set the motor to (between -1.0 and 1.0).
     */
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }
}
