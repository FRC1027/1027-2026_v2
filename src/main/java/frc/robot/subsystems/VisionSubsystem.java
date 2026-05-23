package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawDetection;

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
    private double distToCamera;
    private double distToRobot;
    private double horizontalDistToCamera;
    private boolean hasTarget;

    /* Instance variables to store neural network detection data retrieved from the AprilTag pipeline. */
    private int neuralClassID;
    private double neuralTxnc;
    private double neuralTync;
    private double neuralTa;

    // Stores the last time a target was seen.
    private double lastSeenTime = 0.0;

    /* Constructor for the VisionSubsystem. */
    public VisionSubsystem(String limelightName, int pipelineIndex, int[] desiredTagIDs) {
        this.limelightName = limelightName;
        this.pipelineIndex = pipelineIndex;

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

                    // Assume the camera is pitched up by the mount angle. We rotate the Z distance down to horizontal.
                    double mountAngle = ObjectRecognitionConstants.LIMELIGHT_MOUNT_ANGLE_RADIANS;
                    horizontalDistToCamera = fiducialAdjustedTZ * Math.cos(mountAngle) + fiducialTY * Math.sin(mountAngle);

                    System.out.println("Z Distance: " + fiducialRawTZ + " Adjusted Z Distance: " + fiducialAdjustedTZ + " Horizontal Distance: " + horizontalDistToCamera);

                    // Compute distance from robot to target by subtracting the distance from camera to bumper from the distance from camera to target.
                    distToRobot = distToCamera - ObjectRecognitionConstants.CAMERA_TO_BUMPER_DISTANCE;
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
                    distToCamera = 0.0;
                    distToRobot = 0.0;
                }
            }
        } else if (pipelineIndex == 1) {
            // Get raw neural detector results.
            boolean currentHasTarget = LimelightHelpers.getTV(limelightName); // Do you have a valid target?

            if (currentHasTarget) {
                hasTarget = true;
                lastSeenTime = currentTime;

                RawDetection[] neuralNetworkDetections = LimelightHelpers.getRawDetections(limelightName);
                for (RawDetection detection : neuralNetworkDetections) {
                    neuralClassID = detection.classId;  // Class ID of the detected object
                    neuralTxnc = detection.txnc;        // X offset (no crosshair)
                    neuralTync = detection.tync;        // Y offset (no crosshair)
                    neuralTa = detection.ta;            // Target area\
                }

                // Assume the camera is pitched up by the mount angle. We rotate the Z distance down to horizontal.
                //double mountAngle = ObjectRecognitionConstants.LIMELIGHT_MOUNT_ANGLE_RADIANS;
                //double tzPlanar = tz * Math.cos(mountAngle) + ty * Math.sin(mountAngle);
            } else {
                if (currentTime - lastSeenTime < ObjectRecognitionConstants.LIMELIGHT_TARGET_TIMEOUT) {
                    hasTarget = true;
                } else {
                    // Clear neural network detection data if the target has been lost for longer than the timeout period.
                    hasTarget = false;
                    neuralTxnc = 0.0;
                    neuralTync = 0.0;
                    neuralTa = 0.0;
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

    public double getDistToCamera() {
        return distToCamera;
    }

    public double getDistToRobot() {
        return distToRobot;
    }

    public double getHorizontalDistToCamera() {
        return horizontalDistToCamera;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    /* Getter methods for neural network detection data. */
    public int getNeuralClassID() {
        return neuralClassID;
    }

    public double getNeuralTxnc() {
        return neuralTxnc;
    }

    public double getNeuralTync() {
        return neuralTync;
    }

    public double getNeuralTa() {
        return neuralTa;
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
