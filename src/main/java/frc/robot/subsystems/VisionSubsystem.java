package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Constants.ObjectRecognitionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem {
    /* Instance variable to store the name of the limelight camera to get data from. */
    private String limelightName;
    private int pipelineIndex;

    /* Instance variables to store AprilTag/Fiducial data retrieved from the AprilTag pipeline. */
    private int fiducialID;
    private double fiducialtxnc;
    private double fiducialtync;
    private double fiducialta;
    private double distToCamera;
    private double distToRobot;
    private double ambiguity;
    private boolean hasTarget;
    private int[] desiredTagIDs;

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
        this.desiredTagIDs = desiredTagIDs;

        // Set the initial pipeline index for the Limelight as appropriate.
        if (pipelineIndex == 0) {
            LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex); // Set to AprilTag pipeline
        } else if (pipelineIndex == 1) {
            LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex); // Set to neural network pipeline
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

                RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
                for (RawFiducial fiducial : fiducials) {
                    fiducialID = fiducial.id;               // Tag ID
                    fiducialtxnc = fiducial.txnc;           // X offset (no crosshair)
                    fiducialtync = fiducial.tync;           // Y offset (no crosshair)
                    fiducialta = fiducial.ta;               // Target area
                    distToCamera = fiducial.distToCamera;   // Distance to camera
                    distToRobot = fiducial.distToRobot;     // Distance to robot
                    ambiguity = fiducial.ambiguity;         // Tag pose ambiguity
                    System.out.println("Fiducial ID: " + fiducialID + ", TXNC: " + fiducialtxnc + ", TYNC: " + fiducialtync);
                    System.out.println("DistToCamera: " + distToCamera + ", DistToRobot: " + distToRobot + ", Ambiguity: " + ambiguity);
                }
            } else {
                if (currentTime - lastSeenTime < ObjectRecognitionConstants.LIMELIGHT_TARGET_TIMEOUT) {
                    hasTarget = true;
                } else {
                    // Clear AprilTag/Fiducial data if the target has been lost for longer than the timeout period.
                    hasTarget = false;
                    fiducialID = -1;
                    fiducialtxnc = 0.0;
                    fiducialtync = 0.0;
                    fiducialta = 0.0;
                    distToCamera = 0.0;
                    distToRobot = 0.0;
                    ambiguity = 0.0;
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
                    neuralTa = detection.ta;            // Target area
                }
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
    public int getFiducialID() {
        return fiducialID;
    }

    public double getFiducialtxnc() {
        return fiducialtxnc;
    }

    public double getFiducialtync() {
        return fiducialtync;
    }

    public double getFiducialta() {
        return fiducialta;
    }

    public double getDistToCamera() {
        return distToCamera;
    }

    public double getDistToRobot() {
        return distToRobot;
    }

    public double getAmbiguity() {
        return ambiguity;
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
}
