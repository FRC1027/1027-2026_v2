package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;

/**
 * A command that detects either an AprilTag or a game piece and then either aligns to the target
 * or drives the robot toward it.
 * 
 * This command uses the Limelight camera to detect an AprilTag or game piece and:
 *  - Rotates the robot to face the target.
 *  - Optionally drives forward until a certain distance is reached.
 */
public class DriveTowardTargetCommand extends Command {
    // Instance of the VisionSubsystem to access Limelight data and control pipelines.
    private final VisionSubsystem visionSubsystem;

    // Instance of the SwerveSubsystem to control the robot's movement.
    private final SwerveSubsystem drivebase;

    // Desired stopping distance from bumper to target in meters.
    private final double STOP_DISTANCE = Units.inchesToMeters(48.0);

    private final double maxSpeed = 1.0; // Maximum forward speed in meters per second.
    private final double maxRotation = 1.0; // Maximum rotation speed in radians per second.

    // Distance from the robot's bumper to the target.
    private double distanceToTarget;

    // Variable to store the latest tx value from the Limelight.
    private double tx;

    // Whether to only align to the target without driving forward.
    private boolean alignOnly;

    // Whether a target has been detected.
    private boolean hasTarget;

    // Current forward and rotation speeds.
    private double forwardSpeed = 0.0;
    private double rotationSpeed = 0.0;

    /*
     * Constructor for the DriveTowardTargetCommand. Based on the parameters of the VisionSubsystem,
     * this command will either run in AprilTag detection mode or neural network object detection mode.
     * 
     * @param drivebase the SwerveSubsystem instance used to control the robot's movement.
     * @param visionSubsystem the VisionSubsystem instance used to access Limelight data and control pipelines.
     * @param alignOnly whether to only align to the target without driving toward it.
     */
    public DriveTowardTargetCommand(SwerveSubsystem drivebase, VisionSubsystem visionSubsystem, boolean alignOnly) {
        this.drivebase = drivebase;
        this.visionSubsystem = visionSubsystem;
        this.alignOnly = alignOnly;

        // Require the drivebase so no other drive commands run at the same time.
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        // Runs once when the command starts.

        // Select the appropriate Limelight pipeline for the active detection mode.
        if (visionSubsystem.getPipelineIndex() == ObjectRecognitionConstants.APRIL_TAG_PIPELINE_INDEX) {
            visionSubsystem.setPipelineIndex(ObjectRecognitionConstants.APRIL_TAG_PIPELINE_INDEX);
        } else {
            visionSubsystem.setPipelineIndex(ObjectRecognitionConstants.OBJECT_DETECTION_PIPELINE_INDEX);
        }

        // Reset detection state to avoid stale data from previous runs.
        distanceToTarget = 0.0;
        tx = 0.0;
        hasTarget = false;
    }

    @Override
    public void execute() {
        // Runs repeatedly while the command is active (about every 20 ms).

        if (visionSubsystem.getPipelineIndex() == ObjectRecognitionConstants.APRIL_TAG_PIPELINE_INDEX) {
            // AprilTag detection branch.

            // 1) Validate that a fiducial ID is currently detected.
            double fid = visionSubsystem.getFiducialID();
            if (Double.isNaN(fid) || fid < 0.0) {
                distanceToTarget = 0.0;
                tx = 0.0;
                hasTarget = false;
                stopRobot();
                return;
            }
        } else {
            // Object-detection branch.

            // Get the latest Limelight object-detection results.
            LimelightResults results = LimelightHelpers.getLatestResults(ObjectRecognitionConstants.LIMELIGHT_NAME);

            // 1) Ensure at least one neural-network detection is available.
            if (results.targets_Detector == null || results.targets_Detector.length == 0) {
                distanceToTarget = 0.0;
                tx = 0.0;
                hasTarget = false;
                stopRobot();
                return;
            }
        }

        // --- SHARED DETECTION LOGIC ---

        // Retrieve the Limelight network table to access common entries for both detection modes.
        NetworkTable limelight = visionSubsystem.getLimelight();

        // 2) Check the "tv" flag for target validity.
        double tv = limelight.getEntry("tv").getDouble(0.0);
        if (tv < 1.0) {
            distanceToTarget = 0.0;
            tx = 0.0;
            hasTarget = false;
            stopRobot();
            return;
        }
        hasTarget = true;

        // 3) Read target pose relative to the Limelight camera.
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
        if (pose == null || pose.length < 3) {
            distanceToTarget = 0.0;
            tx = 0.0;
            hasTarget = false;
            stopRobot();
            return;
        }

        // Horizontal offset (left/right) in meters as aiming offset.
        tx = pose[0];

        // Print horizontal offset (tx) before additional offset is applied for debugging.
        System.out.println("Alignment Offset: " + tx);
        
        // Apply an additional offset to the recorded horizontal offset (tx) depending if its offset to the left or right.
        if (tx > 0){
            // If horizontal offset (tx) is to the
            tx = pose[0] + Units.inchesToMeters(3.25);
        } else if (tx < 0){
            // If horizontal offset (tx) is to the
            tx = pose[0] - Units.inchesToMeters(3.25);
        }

        // Calculate bumper-to-target distance from Limelight pose data.
        if (visionSubsystem.getPipelineIndex() == ObjectRecognitionConstants.APRIL_TAG_PIPELINE_INDEX){
            distanceToTarget = visionSubsystem.getFiducialHorizontalDistToRobot();
        } else {
            distanceToTarget = visionSubsystem.getNeuralHorizontalDistToRobot();
        }

        // --- CONTROL LOGIC ---

        // A) Forward speed control.
        // Only drive forward when maxSpeed > 0 and target distance is above stop threshold.
        if (maxSpeed > 0 && distanceToTarget > STOP_DISTANCE) {
            // Proportional approach speed that tapers as we get closer.
            double speedFactor = Math.min(1.0, distanceToTarget / 4.0);
            forwardSpeed = maxSpeed * speedFactor;
        } else {
            forwardSpeed = 0.0;
        }

        // B) Rotation control to reduce horizontal offset (tx).
        double kP_turn = 4.0; // Proportional gain for turning (Increase for faster rotation).
        rotationSpeed = -kP_turn * tx; // Positive tx means target is right, so rotate right (negative Z).

        // Clamp rotation speed to our maximum allowed limit.
        rotationSpeed = Math.max(-maxRotation, Math.min(maxRotation, rotationSpeed));

        // Apply the calculated speeds to the robot. Translation2d(x, y) -> x is forward, y is left.
        drivebase.drive(new Translation2d(forwardSpeed, 0), rotationSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when the command finishes or is interrupted.
        stopRobot();
        SmartDashboard.putString("LL Status/Error Type", interrupted ? "Interrupted" : "Arrived at Target");
        System.out.println("[DriveTowardTarget] Ended");
    }

    @Override
    public boolean isFinished() {
        // Rotation tolerance for considering the robot aligned, measured in radians.
        final double ROTATION_TOLERANCE = 0.1;

        // 1) If we do not currently have a target, stop and let the driver re-trigger.
        if (!hasTarget) {
            return true;
        }

        // 2) In drive mode (maxSpeed > 0), finish once we reach the stop distance.
        boolean reachedDistanceTarget = (maxSpeed > 0) && (distanceToTarget <= STOP_DISTANCE);

        // 3) Finish if aligned within rotation tolerance (align-only mode or while driving).
        boolean alignedTarget = Math.abs(tx) <= ROTATION_TOLERANCE;

        // Command finishes if either we’ve reached distance OR we are aligned.
        return reachedDistanceTarget || alignedTarget;
    }

    /**
     * Helper method to stop the robot completely.
     */
    private void stopRobot() {
        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
    }

    /**
     * Sets the Limelight pipeline to the AprilTag detection pipeline.
     */
    public void setPipelineToAprilTags() {
        LimelightHelpers.setPipelineIndex(ObjectRecognitionConstants.LIMELIGHT_NAME, ObjectRecognitionConstants.APRIL_TAG_PIPELINE_INDEX);
    }

    /**
     * Sets the Limelight pipeline to the object detection pipeline.
     */
    public void setPipelineToObjectDetection() {
        LimelightHelpers.setPipelineIndex(ObjectRecognitionConstants.LIMELIGHT_NAME, ObjectRecognitionConstants.OBJECT_DETECTION_PIPELINE_INDEX);
    }
}
