package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase{
    /* Instance variable to store the name of the limelight camera to get data from. */
    private NetworkTable limelight;
    private String limelightName;
    private int pipelineIndex;

    /* Instance variables to store AprilTag/Fiducial data retrieved from the AprilTag pipeline. */
    private double fiducialID;
    private double fiducialTX;
    private double fiducialTY;
    private double fiducialRawTZ;
    private double fiducialAdjustedTZ;
    private double fiducialDistToCamera;
    private double fiducialHorizontalDistToRobot;
    private double fiducialHorizontalDistToCamera;
    private boolean hasTarget;

    /* Instance variables to store neural network detection data retrieved from the AprilTag pipeline. */
    private String neuralClassName;
    private double neuralConfidence;
    private double neuralTX;
    private double neuralTY;
    private double neuralTZ;
    private double neuralDistToCamera;
    private double neuralHorizontalDistToRobot;
    private double neuralHorizontalDistToCamera;

    // Stores the last time a target was seen.
    private double lastSeenTime = 0.0;

    /* Constructor for the VisionSubsystem. */
    public VisionSubsystem(String limelightName, int pipelineIndex, int[] desiredTagIDs) {
        this.limelightName = limelightName;
        this.pipelineIndex = pipelineIndex;

        // Force the LEDs off initially. Commands can turn them on as needed.
        LimelightHelpers.setLEDMode_ForceOff(limelightName);

        // Set the initial pipeline index for the Limelight as appropriate.
        if (pipelineIndex == 0) {
            LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex); // Set to AprilTag pipeline
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, desiredTagIDs); // Set the desired tag ID filters for the AprilTag pipeline
            limelight = NetworkTableInstance.getDefault().getTable(limelightName);
        } else if (pipelineIndex == 1) {
            LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex); // Set to neural network pipeline
            limelight = NetworkTableInstance.getDefault().getTable(limelightName);
        } else {
            System.out.println("Invalid pipeline index: " + pipelineIndex + ". Defaulting to AprilTag pipeline.");
            LimelightHelpers.setPipelineIndex(limelightName, 0); // Default to AprilTag pipeline
        }
    }

    /* 
     * This method will be called once per scheduler run, retrieving AprilTag/Fiducial data and neural network detection data
     * from the Limelight and storing it in instance variables for use in appropriate subsystems and commands.
     */
    public void periodic() {
        // Gets the current time in seconds.
        double currentTime = Timer.getFPGATimestamp();

        if (pipelineIndex == 0) {
            // Get raw AprilTag/Fiducial data.
            boolean currentHasTarget = LimelightHelpers.getTV(limelightName); // Do you have a valid target?

            if (currentHasTarget) {
                hasTarget = true;
                lastSeenTime = currentTime;

                // Read the target pose in the camera coordinate frame (x = left/right, y = up/down, z = forward).
                double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);

                if (pose.length >= 3){
                    fiducialTX = pose[0]; // X offset from crosshair to target in meters
                    fiducialTY = pose[1]; // Y offset from crosshair to target in meters
                    fiducialRawTZ = Units.metersToInches(pose[2]); // Z distance from camera to target in inches

                    // Adjusted Z distance using a 3rd degree polynomial regression to correct for observed measurement error at longer distances.
                    fiducialAdjustedTZ = -0.0000126596 * Math.pow(fiducialRawTZ, 3) + 0.00572852 * Math.pow(fiducialRawTZ, 2) + 0.311561 * fiducialRawTZ + 24.53905;

                    // Compute the straight-line distance from camera to target using the raw Z distance and the X and Y offsets.
                    fiducialDistToCamera = Math.sqrt(fiducialTX * fiducialTX + fiducialTY * fiducialTY + fiducialRawTZ * fiducialRawTZ);

                    // Assume the camera is pitched up by the mount angle. We rotate the Z distance down to horizontal.
                    double mountAngle = ObjectRecognitionConstants.LIMELIGHT_MOUNT_ANGLE_RADIANS;
                    double zWorld = fiducialRawTZ * Math.cos(mountAngle) - fiducialTY * Math.sin(mountAngle);
                    fiducialHorizontalDistToCamera = Math.sqrt(fiducialTX * fiducialTX + zWorld * zWorld);

                    System.out.println("Z Distance: " + fiducialRawTZ + " Adjusted Z Distance: " + fiducialAdjustedTZ + " Horizontal Distance: " + fiducialHorizontalDistToCamera);

                    // Compute distance from robot to target by subtracting the distance from camera to bumper from the distance from camera to target.
                    fiducialHorizontalDistToRobot = fiducialDistToCamera - ObjectRecognitionConstants.CAMERA_TO_BUMPER_DISTANCE;
                }

                fiducialID = LimelightHelpers.getFiducialID(limelightName); // Fiducial ID of the detected tag
            } else {
                if (currentTime - lastSeenTime < ObjectRecognitionConstants.LIMELIGHT_TARGET_TIMEOUT) {
                    hasTarget = true;
                } else {
                    // Clear AprilTag/Fiducial data if the target has been lost for longer than the timeout period.
                    hasTarget = false;
                    fiducialID = -1.0;
                    fiducialTX = 0.0;
                    fiducialTY = 0.0;
                    fiducialRawTZ = 0.0;
                    fiducialAdjustedTZ = 0.0;
                    fiducialDistToCamera = 0.0;
                    fiducialHorizontalDistToRobot = 0.0;
                    fiducialHorizontalDistToCamera = 0.0;
                }
            }
        } else if (pipelineIndex == 1) {
            // Get raw neural detector results.
            boolean currentHasTarget = LimelightHelpers.getTV(limelightName); // Do you have a valid target?

            if (currentHasTarget) {
                hasTarget = true;
                lastSeenTime = currentTime;

                // Get the latest Limelight object-detection results.
                LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

                if (results.targets_Detector != null && results.targets_Detector.length > 0) {
                    // Store the first detection (typically highest confidence).
                    LimelightTarget_Detector detection = results.targets_Detector[0];
                    neuralClassName = detection.className;
                    neuralConfidence = detection.confidence;

                    // Read the target pose in the camera coordinate frame (x = left/right, y = up/down, z = forward).
                    double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
                    if (pose.length >= 3){
                        neuralTX = pose[0]; // X offset from crosshair to target in meters
                        neuralTY = pose[1]; // Y offset from crosshair to target in meters
                        neuralTZ = pose[2]; // Z distance from camera to target in meters

                        // Compute the straight-line distance from camera to target using the raw Z distance and the X and Y offsets.
                        neuralDistToCamera = Math.sqrt(neuralTX * neuralTX + neuralTY * neuralTY + neuralTZ * neuralTZ);

                        // Assume the camera is pitched up by the mount angle. We rotate the Z distance down to horizontal.
                        double mountAngle = ObjectRecognitionConstants.LIMELIGHT_MOUNT_ANGLE_RADIANS;
                        double zWorld = neuralTZ * Math.cos(mountAngle) - neuralTY * Math.sin(mountAngle);
                        neuralHorizontalDistToCamera = Math.sqrt(neuralTX * neuralTX + zWorld * zWorld);

                        // Compute distance from robot to target by subtracting the distance from camera to bumper from the distance from camera to target.
                        neuralHorizontalDistToRobot = neuralDistToCamera - ObjectRecognitionConstants.CAMERA_TO_BUMPER_DISTANCE;
                    }
                }
            } else {
                if (currentTime - lastSeenTime < ObjectRecognitionConstants.LIMELIGHT_TARGET_TIMEOUT) {
                    hasTarget = true;
                } else {
                    // Clear neural network detection data if the target has been lost for longer than the timeout period.
                    hasTarget = false;
                    neuralClassName = "";
                    neuralConfidence = 0.0;
                    neuralTX = 0.0;
                    neuralTY = 0.0;
                    neuralTZ = 0.0;
                }
            }
        }
    }

    /* Getter methods for AprilTag/Fiducial data. */
    public int getPipelineIndex() {
        return pipelineIndex;
    }

    public double getFiducialID() {
        return fiducialID;
    }

    public double getFiducialTX() {
        return fiducialTX;
    }

    public double getFiducialTY() {
        return fiducialTY;
    }

    public double getFiducialRawTZ() {
        return fiducialRawTZ;
    }

    public double getFiducialAdjustedTZ() {
        return fiducialAdjustedTZ;
    }

    public double getFiducialDistToCamera() {
        return fiducialDistToCamera;
    }

    public double getFiducialHorizontalDistToRobot() {
        return fiducialHorizontalDistToRobot;
    }

    public double getFiducialHorizontalDistToCamera() {
        return fiducialHorizontalDistToCamera;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    /* Getter methods for neural network detection data. */
    public String getNeuralClassName() {
        return neuralClassName;
    }

    public double getNeuralConfidence() {
        return neuralConfidence;
    }

    public double getNeuralTX() {
        return neuralTX;
    }

    public double getNeuralTY() {
        return neuralTY;
    }

    public double getNeuralTZ() {
        return neuralTZ;
    }

    public double getNeuralDistToCamera() {
        return neuralDistToCamera;
    }

    public double getNeuralHorizontalDistToRobot() {
        return neuralHorizontalDistToRobot;
    }

    public double getNeuralHorizontalDistToCamera() {
        return neuralHorizontalDistToCamera;
    }

    /* Setter methods for AprilTags/Fiducial and neural network detection data. */
    public void setPipelineIndex(int pipelineIndex) {
        this.pipelineIndex = pipelineIndex;
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    public void setDesiredTagIDs(int[] desiredTagIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, desiredTagIDs);
    }
}
