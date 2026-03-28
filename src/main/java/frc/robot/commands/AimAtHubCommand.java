package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class AimAtHubCommand extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final boolean isMoving;

    private final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);
    private final double FIELD_LENGTH_METERS = 16.541; 

    private final PIDController thetaController = new PIDController(5.0, 0.0, 0.1);

    public AimAtHubCommand(SwerveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, boolean isMoving) {
        this.swerve = swerve;
        this.translationXSupplier = xSupplier;
        this.translationYSupplier = ySupplier;
        this.isMoving = isMoving;
        
        addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI); 
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        Translation2d targetHub = BLUE_HUB_CENTER;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetHub = new Translation2d(FIELD_LENGTH_METERS - BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY());
        }

        // VIRTUAL TARGET MATH
        if (isMoving) {
            double distanceMeters = currentPose.getTranslation().getDistance(targetHub);
            
            // Tunable constants for shoot-on-the-move
            double timeOfFlight = distanceMeters / SmartDashboard.getNumber("Shooter/BallVelocityMPS", 12.0);   // Estimate: 12 m/s ball speed
            double totalTime = timeOfFlight + SmartDashboard.getNumber("Shooter/SystemLatency", 0.15);          // Estimate: 0.15s system latency
            
            // Pull field velocity directly from YAGSL
            ChassisSpeeds speeds = swerve.getSwerveDrive().getFieldVelocity();
            double deltaX = speeds.vxMetersPerSecond * totalTime;
            double deltaY = speeds.vyMetersPerSecond * totalTime;
            
            // Shift the target opposite of our movement
            targetHub = new Translation2d(targetHub.getX() - deltaX, targetHub.getY() - deltaY);
        }

        Translation2d difference = targetHub.minus(currentPose.getTranslation());
        Rotation2d targetAngle = difference.getAngle();

        double rotationSpeed = thetaController.calculate(
            currentPose.getRotation().getRadians(), 
            targetAngle.getRadians()
        );

        swerve.drive(
            new Translation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble()), 
            rotationSpeed, 
            true 
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}