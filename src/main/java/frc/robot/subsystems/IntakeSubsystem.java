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

import java.util.function.BooleanSupplier;

/**
 * Subsystem that controls the intake motor used to collect game pieces off the floor.
 */
public class IntakeSubsystem extends SubsystemBase {
    // Boolean supplier to check if the hopper is enlarged, allowing for coordinated control between subsystems.
    private final BooleanSupplier isHopperEnlarged;

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
     * 
     * @param isHopperEnlarged supplier that reports whether hopper expansion allows intake operation,
     * enabling coordinated control between the intake and hopper subsystems.
     */
    @SuppressWarnings("removal") // Suppress warnings about deprecated ResetMode and PersistMode usage in SparkMax configuration.
    public IntakeSubsystem(BooleanSupplier isHopperEnlarged) {
        // Store the BooleanSupplier for checking hopper state, enabling dynamic response to hopper enlargement.
        this.isHopperEnlarged = isHopperEnlarged;

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
    public Command continuousIntakeCommand() {
        return runEnd(
            () -> setIntakeSpeed(-0.9), // Run intake at a set speed.
            () -> setIntakeSpeed(0.0) // Stop intake when command is interupted.
        ).onlyWhile(isHopperEnlarged); // If the hopper begins to close while the intake is running, the intake command stops.
    }

    public Command continuousOuttakeCommand() {
        return runEnd(
            () -> setIntakeSpeed(0.9), // Run intake at a set speed.
            () -> setIntakeSpeed(0.0) // Stop intake when command is interupted.
        ).onlyWhile(isHopperEnlarged); // If the hopper begins to close while the intake is running, the intake command stops.
    }

    /**
     * Sets the speed of the intake motor.
     * 
     * @param speed The speed to set the motor to (between -1.0 and 1.0).
     */
    public void setIntakeSpeed(double speed) {
        if (isHopperEnlarged.getAsBoolean()){
            intakeMotor.set(speed);
        } else {
            intakeMotor.set(0.0);
        }
    }
}
