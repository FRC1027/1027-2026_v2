package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.DriveTowardTargetCommand;
import frc.robot.commands.LockWheelsCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ShooterInterpolationTable;
import frc.robot.util.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Set;

/**
 * Subsystem that controls the shooter flywheels and shot execution commands.
 */
public class ShooterSubsystem extends SubsystemBase {
    // Limelight NetworkTable used to fetch target 3D pose data.
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable(ObjectRecognitionConstants.LIMELIGHT_NAME);

    // Reference to the IndexerSubsystem to run the indexer command in parallel with shooting.
    private final IndexerSubsystem m_indexer;

    // Primary shooter motor controller (leader).
    private final TalonFX shooterMotor1;

    // Secondary shooter motor controller (follower).
    private final TalonFX shooterMotor2;

    // The follow object that controls the 2nd motor's behavior
    private final StrictFollower followerRequest = new StrictFollower(ShooterConstants.SHOOTER_MOTOR_ID1);

    // --- NEW: Odometry Targeting Variables ---
    private SwerveSubsystem m_drivebase;
    private final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);
    private final double FIELD_LENGTH_METERS = 16.541;

    /**
     * Creates the shooter subsystem, configures TalonFX control gains, and sets up
     * follower behavior and dashboard tuning entries.
     * 
     * @param m_indexer a reference to the IndexerSubsystem for use in parallel control of the subsystems.
     */
    public ShooterSubsystem(IndexerSubsystem m_indexer) {
        // Store the reference to the IndexerSubsystem for use in shooting commands.
        this.m_indexer = m_indexer;

        // Initialize the shooter motors using configured CAN IDs.
        shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID1);
        shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID2);

        // Configure velocity-loop gains and neutral mode for consistent flywheel behavior.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.15; // Proportional gain for velocity control (tuning may be required).
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMotor1.getConfigurator().apply(config);

        // Set Motor 2 to run physically opposed to Motor 1 on the shared shaft.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2.getConfigurator().apply(config);

        // Publish the radius efficiency to SmartDashboard so it can be tuned live.
        SmartDashboard.putNumber("Shooter/VelocityEfficiency", ShooterConstants.VELOCITY_EFFICIENCY);

        // Publish a changable shooter to target distance value in the case that the AprilTag is not visible.
        SmartDashboard.putNumber("Shooter/TunableDistance", ShooterConstants.MINIMUM_DISTANCE);

        // Publish the options for RPS calculation to Elastic Dashboard. Default option uses projectile motion calculations.
        SendableChooser<String> chooser = new SendableChooser<String>();
        chooser.setDefaultOption("RPS Projectile Motion", "RPS Projectile Motion");
        chooser.addOption("RPS Interpolation Table", "RPS Interpolation Table");
        chooser.addOption("Averaged RPS", "Averaged RPS");
        SmartDashboard.putData("Shooter/RPSCalculationType", chooser);
    }

    /**
     * Calculates the required launch RPS using projectile motion physics.
     * 
     * @param shooterToTag The precise distance to the target in meters.
     * @return Theoretical required RPS, or 0.0 if mathematically invalid.
     */
    public double calculateTheoreticalRPS(double shooterToTag) {
        // Calculates the distance from the shooter to the AprilTag.
        // If shooterToTag is 0.0, then calculate distance using Limelight data. Otherwise, use parameter value.
        if (shooterToTag == 0.0) {
            shooterToTag = getDistance();
        }

        // Return NaN if the distance data is missing or invalid.
        if (!Double.isFinite(shooterToTag)) {
            return 0.0;
        }

        // If the target is too high relative to the launch angle, the projectile motion equation would be invalid.
        if (shooterToTag * Math.tan(ShooterConstants.SHOOTER_ANGLE) <= ShooterConstants.HEIGHT_DIFFERENCE) {
            return 0.0;
        }

        // Calculate required launch velocity using projectile motion (ignoring air resistance).
        double velocity = Math.sqrt((ShooterConstants.GRAVITY_CONSTANT * shooterToTag * shooterToTag) / 
                                    (2 * Math.cos(ShooterConstants.SHOOTER_ANGLE) * Math.cos(ShooterConstants.SHOOTER_ANGLE) * 
                                    (shooterToTag * Math.tan(ShooterConstants.SHOOTER_ANGLE) - ShooterConstants.HEIGHT_DIFFERENCE)));

        // Allows for tuning the efficiency factor via the SmartDashboard to improve accuracy of RPS calculations.
        double efficiency = SmartDashboard.getNumber("Shooter/VelocityEfficiency", ShooterConstants.VELOCITY_EFFICIENCY);

        // Calculates the effective velocity of the shooter wheel by applying an efficiency factor to account for real-world conditions.
        double adjustedVelocity = velocity * efficiency;
        
        // Convert linear velocity to wheel RPS using the effective velocity value.
        double rps = adjustedVelocity / (2 * Math.PI * ShooterConstants.SHOOTER_WHEEL_RADIUS);

        // Return the calculated wheel speed in revolutions per second (Multiply by Compression Loss Compensation).
        return rps * 1.03;
    }

    /**
     * Calculate the required launch RPS using the data from the Interpolation Table.
     * 
     * @param shooterToTag the measured distance from the shooter to the AprilTag.
     * @return the RPS recorded in the Interpolation Table.
     */
    public double calculateTableRPS(double shooterToTag) {
        double tableRPS = ShooterInterpolationTable.getRPS(shooterToTag);
        return tableRPS;
    }

    /**
     * Average both the calculated projectile motion RPS and the calculated Interpolation Table RPS.
     * 
     * @param shooterToTag the measured distance from the shooter to the AprilTag.
     * @return the averaged RPS to run the flywheels.
     */
    public double calculateAveragedRPS(double shooterToTag) {
        // Get both the theoretical RPS and the empirical table RPS.
        double theoreticalRPS = calculateTheoreticalRPS(0.0);
        double tableRPS = ShooterInterpolationTable.getRPS(shooterToTag);

        // Blend them via a tunable weight (1.0 = 100% table, 0.0 = 100% theoretical). Default is 0.5 (50/50 split).
        double tableWeight = SmartDashboard.getNumber("Shooter/TableWeight", 0.5);
        
        return (tableRPS * tableWeight) + (theoreticalRPS * (1.0 - tableWeight));
    }

    /**
     * Sets the shooter motor to a RPS calculated via relevant data from Limelight.
     */
    public void setShooterRPS() {
        double wheelRPS = 0.0;

        if (SmartDashboard.getString("Shooter/RPSCalculationType", "RPS Projectile Motion").equals("RPS Projectile Motion")) {
            wheelRPS = calculateTheoreticalRPS(0.0);
        } else if (SmartDashboard.getString("Shooter/RPSCalculationType", "RPS Projectile Motion").equals("RPS Interpolation Table")) {
            wheelRPS = calculateTableRPS(getDistance());
        } else if (SmartDashboard.getString("Shooter/RPSCalculationType", "RPS Projectile Motion").equals("Averaged RPS")) {
            wheelRPS = calculateAveragedRPS(getDistance());
        }

        // If the calculated RPS is not valid or equal to 0, stop the motor to avoid unintentional behavior.
        if (!Double.isFinite(wheelRPS) || wheelRPS == 0.0) {
            shooterMotor1.setControl(new NeutralOut());
            shooterMotor2.setControl(followerRequest);
            return;
        }

        // Convert wheel RPS to motor RPS using the configured gear ratio.
        double motorRPS = wheelRPS * ShooterConstants.GEAR_RATIO;

        // Apply the RPS to the shooter motors.
        shooterMotor1.setControl(new VelocityVoltage(motorRPS));
        shooterMotor2.setControl(followerRequest);
    }

    /**
     * Aligns the robot to the target tag and shoots if the tag ID is valid.
     * Valid tag IDs are in front of Red Hub (9 or 10), in front of Blue Hub (25 or 26), or 4 for testing in the RCH.
     *
     * @param drivebase swerve subsystem used to drive toward the tag
     * @return command that aligns to the tag and then shoots, or no-op if tag is invalid
     */
    public Command shootAlign(SwerveSubsystem drivebase) {
        return Commands.defer(() -> {
            double fid = LimelightHelpers.getFiducialID(ObjectRecognitionConstants.LIMELIGHT_NAME);

            if (fid == 4 || fid == 10 || fid == 26) {
                return new DriveTowardTargetCommand(drivebase, 0.0, 2.0) // Aligns to the target tag using only rotational movement (max speed = 0)
                        // Once the alignment command finishes, run the shoot command while also locking the wheels to prevent movement during shooting.
                        .andThen(Commands.deadline(
                                shoot(), // End the command when the shoot command finishes (which is when the driver releases the trigger)
                                new LockWheelsCommand(drivebase).repeatedly()));
            }
            // If no valid tag is seen, return a "do-nothing" command to avoid unintended motion.
            return Commands.none();
        }, Set.of(this, drivebase));
    }

    /**
     * Runs the shooter at varying speeds (dependant on calculated distance) for a certain period of time, then stops.
     *
     * @return a command that spins the shooter using Limelight-based RPS until interrupted, in parallel with
     * the indexer that feeds balls into the shooter, then stops the shooter and indexer when the command ends.
     */
    public Command shoot() {
        return Commands.deadline(
            runEnd(
                this::setShooterRPS, // Continuously update the shooter RPS based on Limelight data while the command is active.
                () -> {
                    shooterMotor1.setControl(new NeutralOut()); // Stop the shooter motor when the command ends.
                    shooterMotor2.setControl(followerRequest);
                }
            ), 
            m_indexer.runIndexerCommand() // Run the indexer command in parallel to feed balls into the shooter while shooting. The indexer will stop when the shoot command ends.
        ).until(() -> calculateTheoreticalRPS(0.0) == 0.0);
    }

    /**
     * Runs the shooter at a fixed speed for a certain period of time, then stops. Intended for shooting when
     * the robot is pressed against the base of the hub.
     * 
     * @return a command that runs the shooter at a fixed speed until interrupted.
     */
    public Command shootClose() {
        return Commands.deadline(
            runEnd(
                () -> {
                    shooterMotor1.setControl(new VelocityVoltage(calculateTheoreticalRPS(Units.inchesToMeters(16)))); // UPDATE DISTANCE AFTER TESTING
                    shooterMotor2.setControl(followerRequest);
                },
                () -> {
                    shooterMotor1.setControl(new NeutralOut());
                    shooterMotor2.setControl(followerRequest);
                }
            ),
            m_indexer.runIndexerCommand()
        );
    }

    /**
     * A method used for shooter testing to run the indexer and shooter at full speed without Limelight distance calculation.
     * 
     * @return a command that runs the shooter at full speed in parallel with the intake.
     */
    public Command fullSpeed(){
        return Commands.deadline(
            runEnd(
                () -> setShooterSpeed(1.0), 
                () -> {
                    shooterMotor1.setControl(new NeutralOut());
                    shooterMotor2.setControl(followerRequest);
                }
            ),
            m_indexer.runIndexerCommand());
    }

    /**
     * Sets the shooter motor speed for manual control. Positive speed shoots out,
     * negative speed intakes in, and zero stops the motor.
     *
     * @param speed motor output in the [-1, 1] range (sign controls direction)
     */
    public void setShooterSpeed(double speed) {
        shooterMotor1.set(speed);
        shooterMotor2.setControl(followerRequest);
    }

    /**
     * Calculate the distance from the shooter to the AprilTag.
     * 
     * @return the distance from the shooter to the AprilTag.
     */
    public double getDistance() {
        // Calculate the distance from the shooter to the target tag using Limelight data.
        return Utils.calculateDistanceToTarget(limelight);
    }

    // ========================================================================
    // NEW ODOMETRY TARGETING METHODS (Added for dry-coding/testing)
    // ========================================================================

    public void setDrivebase(SwerveSubsystem drivebase) {
        this.m_drivebase = drivebase;
        SmartDashboard.putNumber("Shooter/SweetSpotOffset", 0.3);
        SmartDashboard.putNumber("Shooter/BallVelocityMPS", 12.0);
        SmartDashboard.putNumber("Shooter/SystemLatency", 0.15);
    }

    public double getAdjustedTargetDistance(boolean isMoving) {
        if (m_drivebase == null) return 0.0;

        double sweetSpotOffsetMeters = SmartDashboard.getNumber("Shooter/SweetSpotOffset", 0.3);
        double ballVelocityMPS = SmartDashboard.getNumber("Shooter/BallVelocityMPS", 12.0);
        double systemLatency = SmartDashboard.getNumber("Shooter/SystemLatency", 0.15);
        Translation2d targetHub = BLUE_HUB_CENTER;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetHub = new Translation2d(FIELD_LENGTH_METERS - BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY());
        }

        Pose2d currentPose = m_drivebase.getPose();

        // --- NEW: VIRTUAL TARGET MATH ---
        if (isMoving) {
            double initialDistance = currentPose.getTranslation().getDistance(targetHub);
            double timeOfFlight = initialDistance / ballVelocityMPS; 
            double totalTime = timeOfFlight + systemLatency;
            
            // Using WPILib's fully-qualified path
            ChassisSpeeds speeds = m_drivebase.getSwerveDrive().getFieldVelocity();
            double deltaX = speeds.vxMetersPerSecond * totalTime;
            double deltaY = speeds.vyMetersPerSecond * totalTime;
            
            targetHub = new Translation2d(targetHub.getX() - deltaX, targetHub.getY() - deltaY);
        }

        double finalDistanceToCenter = currentPose.getTranslation().getDistance(targetHub);
        return Math.max(0.0, finalDistanceToCenter - sweetSpotOffsetMeters);
    }

    public double calculateWheelRPSFromOdometry(boolean isMoving) {
        double shooterToTarget = getAdjustedTargetDistance(isMoving);
        
        if (!Double.isFinite(shooterToTarget) || shooterToTarget == 0.0) {
            return 0.0;
        }

        if (shooterToTarget * Math.tan(ShooterConstants.SHOOTER_ANGLE) <= ShooterConstants.HEIGHT_DIFFERENCE) {
            return 0.0;
        }

        double velocity = Math.sqrt((ShooterConstants.GRAVITY_CONSTANT * shooterToTarget * shooterToTarget) / 
                                    (2 * Math.cos(ShooterConstants.SHOOTER_ANGLE) * Math.cos(ShooterConstants.SHOOTER_ANGLE) * (shooterToTarget * Math.tan(ShooterConstants.SHOOTER_ANGLE) - ShooterConstants.HEIGHT_DIFFERENCE)));

        double efficiency = SmartDashboard.getNumber("Shooter/VelocityEfficiency", ShooterConstants.VELOCITY_EFFICIENCY);
        double adjustedVelocity = velocity * efficiency;
        double rps = adjustedVelocity / (2 * Math.PI * ShooterConstants.SHOOTER_WHEEL_RADIUS);

        return rps * 1.03;
    }

    public void setShooterRPSFromOdometry(boolean isMoving) {
        double wheelRPS = calculateWheelRPSFromOdometry(isMoving);

        if (!Double.isFinite(wheelRPS) || wheelRPS == 0.0) {
            shooterMotor1.setControl(new NeutralOut());
            shooterMotor2.setControl(followerRequest);
            return;
        }

        double motorRPS = wheelRPS * ShooterConstants.GEAR_RATIO;
        shooterMotor1.setControl(new VelocityVoltage(motorRPS));
        shooterMotor2.setControl(followerRequest); // ensure StrictFollower spelling is correct based on your code
    }

    public Command shootOdometry(boolean isMoving) {
        return Commands.deadline(
            runEnd(
                () -> setShooterRPSFromOdometry(isMoving), 
                () -> {
                    shooterMotor1.setControl(new NeutralOut()); 
                    shooterMotor2.setControl(followerRequest);
                }
            ), 
            m_indexer.runIndexerCommand() 
        ).until(() -> calculateWheelRPSFromOdometry(isMoving) == 0.0);
    }
}
