package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
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

    // Encoder for the hopper motor.
    private final RelativeEncoder hopperEncoder;

    // PID Controller for active position holding.
    private final SparkClosedLoopController hopperPIDController;

    // Motor configuration for the hopper motor.
    public static final SparkMaxConfig hopperConfig = new SparkMaxConfig();

    static {
        hopperConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);

        // Configure PID constants for active holding.
        // P-gain tuning required!
        hopperConfig.closedLoop
            .pid(0.1, 0.0, 0.0)
            .outputRange(-0.5, 0.5); // Limit effort to half power to avoid burning out the motor.
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

        // Get the encoder associated with the hopperMotor.
        hopperEncoder = hopperMotor.getEncoder();

        // Get the PID controller associated with the hopperMotor.
        hopperPIDController = hopperMotor.getClosedLoopController();

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
            // Get the current position of the hopperMotor from the encoder.
            double initialPosition = hopperEncoder.getPosition();

            // The arm needs to move 90 degrees, which is 0.25 of a full rotation (360 degrees).
            double targetArmRotations = 90.0 / 360.0;

            // Number of motor rotations required to fully extend/retract the hopper arm by 90 degrees.
            // The motor has an 81:1 gear ratio, meaning the motor turns 81 times for every 1 turn of the arm.
            double targetRotations = targetArmRotations * HopperConstants.HOPPER_GEAR_RATIO;

            if (!hopperEnlarged) {
                System.out.println("Hopper is Extended: " + hopperEnlarged);

                // Run the hopper forward until the encoder reads initialPosition + targetRotations.
                return run(() -> setHopperSpeed(0.5))
                        .until(() -> hopperEncoder.getPosition() >= initialPosition + targetRotations)
                        .finallyDo(() -> {
                            setHopperSpeed(0.0);
                            //holdPosition(initialPosition + targetRotations);
                            hopperEnlarged = true;
                            System.out.println("Hopper is Extended: " + hopperEnlarged);
                        });
            } else {
                System.out.println("Hopper is Extended: " + hopperEnlarged);

                // Run the hopper in reverse until the encoder reads initialPosition - targetRotations.
                return run(() -> setHopperSpeed(-0.5))
                        .until(() -> hopperEncoder.getPosition() <= initialPosition - targetRotations)
                        .finallyDo(() -> {
                            setHopperSpeed(0.0);
                            //holdPosition(initialPosition - targetRotations);
                            hopperEnlarged = false;
                            System.out.println("Hopper is Extended: " + hopperEnlarged);
                        });
            }
        }, Set.of(this));
    }

    /**
     * Actively holds the hopper at a given position using the PID controller.
     *
     * @param targetPosition the encoder position to hold at
     */
    @SuppressWarnings("removal") // Suppress deprecation warning if old REVLib version requires it, but use new format if possible.
    public void holdPosition(double targetPosition) {
        // Fall back to old method if ClosedLoopSlot isn't imported, but the lint showed it's deprecated. Let's use the new one.
        hopperPIDController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
