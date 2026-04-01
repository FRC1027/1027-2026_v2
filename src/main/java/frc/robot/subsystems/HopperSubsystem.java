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
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.RobotContainer;
import frc.robot.util.Constants.HopperConstants;

import java.util.Set;

/**
 * Subsystem that controls the hopper motor used to enlarge the holding space for the balls.
 */
public class HopperSubsystem extends SubsystemBase {
    // Boolean to indicate whether the hopper is currently expanded, for use in state management.
    private boolean hopperEnlarged;

    // Hopper motor.
    private final SparkMax hopperMotor;

    // Motor configuration for the hopper motor.
    public static final SparkMaxConfig hopperConfig = new SparkMaxConfig();

    static {
        hopperConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    }

    /**
     * Creates the hopper subsystem and configures the SparkMax motor.
     */
    @SuppressWarnings("removal") // Suppress warnings about deprecated ResetMode and PersistMode usage in SparkMax configuration.
    public HopperSubsystem() {
        // Default to not enlarged state on initialization; the hopper starts in its normal configuration.
        hopperEnlarged = false;

        // Initialize the hopper motor using configured CAN ID.
        hopperMotor = new SparkMax(HopperConstants.HOPPER_MOTOR_ID1, MotorType.kBrushless);

        // Configure the hopper motor using safe parameter reset and persistent parameter storage.
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Manual hopper control using driver bumpers:
     * Right bumper expands the hopper, left bumper retracts the bumper, neither stops.
     * 
     * KNOWN ISSUE: This command does not update the hopperEnlarged state variable.
     */
    public Command manualHopperControl() {
        return runEnd(() -> {
            Trigger rightBumper = RobotContainer.mechXbox.rightBumper();
            Trigger leftBumper = RobotContainer.mechXbox.leftBumper();

            if (rightBumper.getAsBoolean()) {
                setHopperSpeed(0.7); // Run hopper forward while right bumper is held.
            } else if (leftBumper.getAsBoolean()) {
                setHopperSpeed(-0.7); // Run hopper in reverse while left bumper is held.
            } else {
                setHopperSpeed(0.0); // Stop when neither bumper is pressed.
            }
        }, () -> setHopperSpeed(.0)); // Ensure motors stop when the command ends
    }

    /**
     * Enlarges or retracts the hopper based on its current state. If the hopper is not enlarged, 
     * it will run the motor to enlarge it; if it is already enlarged, it will run the motor in 
     * reverse to retract it.
     */
    public Command hopperEnlarger2000Command() {
        return Commands.defer(() -> {
            if (!hopperEnlarged) {
                System.out.println("State of Hopper: " + hopperEnlarged);

                // Example: run the hopper at 50% speed for 2 seconds to fully enlarge (tuning is required)
                return run(() -> setHopperSpeed(0.5))
                        .withTimeout(2.0) // Time how long it takes to fully extend the hopper with a known speed
                        .finallyDo(() -> {
                            setHopperSpeed(0.0);
                            hopperEnlarged = true;
                            System.out.println("State of Hopper: " + hopperEnlarged);
                        });
            } else {
                System.out.println("State of Hopper: " + hopperEnlarged);

                // Example: run the hopper at 50% speed for 2 seconds to fully enlarge (tuning is required)
                return run(() -> setHopperSpeed(-0.5))
                        .withTimeout(2.0) // Time how long it takes to fully retract the hopper with a known speed
                        .finallyDo(() -> {
                            setHopperSpeed(0.0);
                            hopperEnlarged = false;
                            System.out.println("State of Hopper: " + hopperEnlarged);
                        });
            }
        }, Set.of(this));
    }

    /**
     * Sets hopper motor output in the [-1, 1] range.
     *
     * @param speed motor output percent (sign controls direction)
     */
    public void setHopperSpeed(double speed) {
        hopperMotor.set(speed); // Set the speed of the hopper motor
    }

    /**
     * Returns the current state of the hopper (enlarged or not). This is used for state management in commands and
     * other subsystems.
     * 
     * @return true if the hopper is enlarged, false otherwise
     */
    public boolean getHopperEnlarged() {
        return hopperEnlarged;
    }
}
