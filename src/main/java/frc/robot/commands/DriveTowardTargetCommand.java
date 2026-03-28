package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.Utils;

/**
 * A command that detects either an AprilTag or a game piece and then either aligns to the target
 * or drives the robot toward it.
 * 
 * This command uses the Limelight camera to detect an AprilTag or game piece and:
 *  - Rotates the robot to face the target.
 *  - Optionally drives forward until a certain distance is reached.
 */
public class DriveTowardTargetCommand extends Command {

    /**
     * Helper class to store detection state of the detected game piece.
     */
    private static class DetectionState {
        //public String className = ""; // Name of the detected game piece
        //public double confidence = 0.0; // Confidence of the detection
        public double distance = 0.0; // Calculated distance in meters
        public boolean hasTarget = false; // Whether a target was detected

        /**
         * Clears the detection state.
         */
        public void clear() {
            //className = "";
            //confidence = 0.0;
            distance = 0.0;
            hasTarget = false;
        }
    }

    // Current detection state of the detected game piece.
    private final DetectionState currentState = new DetectionState();

    // Limelight NetworkTable used to fetch pose/target state values for control.
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable(ObjectRecognitionConstants.LIMELIGHT_NAME);

    // Instance of the SwerveSubsystem to control the robot's movement.
    private final SwerveSubsystem drivebase;

    // Maximum forward speed limit (m/s).
    private final double maxSpeed;

    // Maximum rotation speed limit (rad/s).
    private final double maxRotation;

    // If true, run AprilTag mode; if false, run neural object detection mode.
    private boolean detectAprilTag = true;

    // Desired stopping distance from bumper to target in meters.
    private double STOP_DISTANCE = 0.5;

    // Current forward and rotation speeds.
    private double forwardSpeed = 0.0;
    private double rotationSpeed = 0.0;

    // Variable to store the latest tx value from the Limelight.
    private double tx;

    /**
     * Constructor for DriveTowardTargetCommand with explicit speed limits.
     * This constructor does not set detection mode, so the command uses the field default
     * (`detectAprilTag == false`) unless it is changed elsewhere.
     * 
     * @param drivebase   The swerve drive subsystem used to move the robot.
     * @param maxSpeed    The maximum forward speed in meters per second. Set to 0 for align-only.
     * @param maxRotation The maximum rotation speed in radians per second.
     */
    public DriveTowardTargetCommand(SwerveSubsystem drivebase, double maxSpeed, double maxRotation) {
        this.drivebase = drivebase;
        this.maxSpeed = maxSpeed;
        this.maxRotation = maxRotation;

        // Require the drivebase so no other drive commands run at the same time.
        addRequirements(drivebase);
    }

    /**
     * Constructor for DriveTowardTargetCommand that allows specifying whether to detect AprilTags or
     * use object detection. It uses default max speeds (2.0 m/s for forward, 2.0 rad/s for rotation).
     * 
     * @param drivebase      The swerve drive subsystem.
     * @param detectAprilTag If true, uses AprilTag detection; if false, uses object detection.
     */
    public DriveTowardTargetCommand(SwerveSubsystem drivebase, boolean detectAprilTag) {
        this.drivebase = drivebase;
        this.maxSpeed = 2.0;
        this.maxRotation = 2.0;
        this.detectAprilTag = detectAprilTag;

        if (!detectAprilTag) {
            // If using object detection, we want to use a closer stop distance since we want to be right on top of the game piece to pick it up.
            // We can adjust this as needed based on testing.
            STOP_DISTANCE = 0.1; // In meters
        }

        // Require the drivebase so no other drive commands run at the same time.
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        // Runs once when the command starts.

        // Select the appropriate Limelight pipeline for the active detection mode.
        if (detectAprilTag) {
            setPipelineToAprilTags();
        } else {
            setPipelineToObjectDetection();
        }

        // Reset detection state to avoid stale data from previous runs.
        currentState.clear();
    }

    @Override
    public void execute() {
        // Runs repeatedly while the command is active (about every 20 ms).

        if (detectAprilTag) {
            // AprilTag detection branch.

            // 1) Validate that a fiducial ID is currently detected.
            double fid = LimelightHelpers.getFiducialID(ObjectRecognitionConstants.LIMELIGHT_NAME);
            if (Double.isNaN(fid) || fid < 0.0) {
                currentState.clear();
                stopRobot();
                return;
            }
        } else {
            // Object-detection branch.

            // Get the latest Limelight object-detection results.
            LimelightResults results = LimelightHelpers.getLatestResults(ObjectRecognitionConstants.LIMELIGHT_NAME);

            // 1) Ensure at least one neural-network detection is available.
            if (results.targets_Detector == null || results.targets_Detector.length == 0) {
                currentState.clear();
                stopRobot();
                return;
            }

            // Store the first detection (typically highest confidence).
            //LimelightTarget_Detector detection = results.targets_Detector[0];
            //currentState.className = detection.className;
            //currentState.confidence = detection.confidence;
        }

        // --- SHARED DETECTION LOGIC ---

        // 2) Check the "tv" flag for target validity.
        double tv = limelight.getEntry("tv").getDouble(0.0);
        if (tv < 1.0) {
            currentState.clear();
            stopRobot();
            return;
        }

        // 3) Read target pose relative to the Limelight camera.
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
        if (pose == null || pose.length < 3) {
            currentState.clear();
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
        currentState.distance = Utils.calculateDistanceToTarget(limelight);
        currentState.hasTarget = true;

        // --- CONTROL LOGIC ---

        // A) Forward speed control.
        // Only drive forward when maxSpeed > 0 and target distance is above stop threshold.
        if (maxSpeed > 0 && currentState.distance > STOP_DISTANCE) {
            // Proportional approach speed that tapers as we get closer.
            double speedFactor = Math.min(1.0, currentState.distance / 4.0);
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
        if (!currentState.hasTarget) {
            return true;
        }

        // 2) In drive mode (maxSpeed > 0), finish once we reach the stop distance.
        boolean reachedDistanceTarget = (maxSpeed > 0) && (currentState.distance <= STOP_DISTANCE);

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
