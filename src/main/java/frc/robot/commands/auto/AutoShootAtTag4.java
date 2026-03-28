package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

// (Optional) Use WPILib's official AprilTag field layout instead of a Limelight-only pipeline.
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;

/**
 * Autonomous routine to locate AprilTag ID 4 and shoot.
 *
 * Sequence:
 * 1. Drive forward about 1 foot to improve initial visibility.
 * 2. Approach AprilTag ID 4 with Limelight until the bumper is about 1.5 m away.
 * 3. Stop and hold position.
 * 4. Pause briefly to stabilize aim.
 * 5. Fire using {@link ShooterSubsystem#shoot()}.
 */
public class AutoShootAtTag4 extends SequentialCommandGroup {

    /**
     * Builds the autonomous sequence that drives to and shoots at AprilTag 4.
     *
     * @param drivebase swerve subsystem used for autonomous motion
     * @param shooter shooter subsystem used to fire game pieces
     */
    public AutoShootAtTag4(SwerveSubsystem drivebase, ShooterSubsystem shooter) {
        addCommands(

            // Step 1: Drive forward a short distance (~1 ft) to improve initial tag visibility.
            Commands.run(() -> drivebase.drive(
                        new Translation2d(0.25, 0.0), // Forward velocity of 0.25 m/s.
                        0.0, // No rotation.
                        true // Field-relative.
                    ), drivebase)
                    .withTimeout(Units.feetToMeters(1)) // Run long enough to cover about 1 ft.
                    .andThen(() -> drivebase.drive( // Then stop.
                        new Translation2d(0.0, 0.0),
                        0.0,
                        true
                    )),

            // Step 2: Approach AprilTag 4 using Limelight until bumper distance is about 1.5 m.
            Commands.run(() -> {
                // Continue only when the currently tracked fiducial is tag 4.
                if (LimelightHelpers.getFiducialID("limelight") == 4) {
                    System.out.println("tracking id 4");

                    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

                    // Read the target-valid flag ("tv"):
                    // - tv = 1 -> valid target in view.
                    // - tv = 0 -> no valid target.
                    double tv = limelight.getEntry("tv").getDouble(0.0);
                    if (tv < 1.0) {
                        SmartDashboard.putString("LL Status", "No target");
                        return;
                    }

                    // Validate camera-space pose data (x, y, z).
                    double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
                    if (pose == null || pose.length < 3) {
                        SmartDashboard.putString("LL Status", "No pose array");
                        return;
                    }

                    double tx = pose[0]; // Horizontal offset (m).
                    double ty = pose[1]; // Vertical offset (m).
                    double tz = pose[2]; // Forward distance from camera to tag (m).

                    // Compute camera-to-tag straight-line distance (Euclidean norm).
                    double cameraToTagDist = Math.sqrt(tx * tx + ty * ty + tz * tz);

                    // Convert camera-to-tag distance into bumper-to-tag distance using camera offset.
                    double camToBumper = 0.3302; // Measure this on your robot (meters).
                    double bumperToTagDist = Math.max(0.0, cameraToTagDist - camToBumper);

                    // Publish camera-space values to SmartDashboard for debugging.
                    SmartDashboard.putNumber("LL tx (m)", tx);
                    SmartDashboard.putNumber("LL ty (m)", ty);
                    SmartDashboard.putNumber("LL tz (m)", tz);
                    SmartDashboard.putNumber("LL camera->tag (m)", cameraToTagDist);
                    SmartDashboard.putNumber("LL bumper->tag (m)", bumperToTagDist);

                    // Stop threshold (1.5 m from bumper).
                    double stopDistance = 1.5;

                    if (bumperToTagDist >= stopDistance) {
                        System.out.println(bumperToTagDist);
                        drivebase.drive(new Translation2d(0.25, 0.0), 0.0, true);
                    } else {
                        System.out.println(bumperToTagDist);
                        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
                    }
                } else {
                    System.out.println("id not found");
                    drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
                }

                /* --- OPTIONAL FIELD LAYOUT VERSION ---
                 * Uncomment this block and comment out the above Limelight chase code
                 * if using WPILib official field layout instead of paper tag.
                 *
                 * AprilTagFieldLayout fieldLayout = AprilTagFields.k2025Crescendo.loadAprilTagLayoutField();
                 * Pose3d tagPose = fieldLayout.getTagPose(4).get();  // ID 4 pose on field
                 * drivebase.driveToPose(tagPose.toPose2d());
                 * turret.trackTargetWithLimelight(); // Align turret once at tag
                 */
            }, drivebase).withTimeout(10.0),

            // Step 3: Stop drivebase fully.
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true), drivebase),

            // Step 4: Small pause to stabilize aim before firing.
            new WaitCommand(0.3),

            // Step 5: Shoot at AprilTag 4.
            shooter.shoot()

            // --- OPTIONAL FAILSAFE ---
            // If AprilTag 4 is not found within X seconds, skip to shooting anyway:
            // .deadlineWith(new WaitCommand(3.0).andThen(shooter.TimedOuttake()))
        );
    }
}
