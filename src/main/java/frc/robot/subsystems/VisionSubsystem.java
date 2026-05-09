package frc.robot.subsystems;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem {
    private int id;
    private double txnc;
    private double tync;
    private double ta;
    private double distToCamera;
    private double distToRobot;
    private double ambiguity;

    public VisionSubsystem() {

    }

    public void periodic() {
        // This method will be called once per scheduler run
        // Get raw AprilTag/Fiducial data
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        for (RawFiducial fiducial : fiducials) {
            id = fiducial.id;                 // Tag ID
            txnc = fiducial.txnc;             // X offset (no crosshair)
            tync = fiducial.tync;             // Y offset (no crosshair)
            ta = fiducial.ta;                 // Target area
            distToCamera = fiducial.distToCamera;  // Distance to camera
            distToRobot = fiducial.distToRobot;    // Distance to robot
            ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
            System.out.println("Fiducial ID: " + id + ", TXNC: " + txnc + ", TYNC: " + tync);
            System.out.println("DistToCamera: " + distToCamera + ", DistToRobot: " + distToRobot + ", Ambiguity: " + ambiguity);
        }
    }
}
