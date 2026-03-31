package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Command that locks the swerve modules in place to resist robot movement.
 */
public class LockWheelsCommand extends Command {
    private final SwerveSubsystem swerve;

    /**
     * Creates a wheel-lock command for the provided swerve subsystem.
     *
     * @param swerve the drivebase to lock
     */
    public LockWheelsCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    /**
     * Publishes lock state when the command starts.
     */
    @Override
    public void initialize() {
        // Runs once when the command is scheduled.
        SmartDashboard.putString("Wheel Lock Status", "Wheels are Now Locked");
    }

    /**
     * Continuously commands the wheel-lock stance while active.
     */
    @Override
    public void execute() {
        // Runs repeatedly while the command is active (about every 20 ms).
        swerve.lock();

        SmartDashboard.putString("Wheel Lock Status", "Wheels are Locked");
    }

    /**
     * Publishes unlock state when the command ends.
     *
     * @param interrupted true if the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        // Runs when the command finishes or is interrupted.
        SmartDashboard.putString("Wheel Lock Status", "Wheels are Now Unlocked");
    }

    /**
     * Keeps the command running until it is externally interrupted.
     *
     * @return always false
     */
    @Override
    public boolean isFinished() {
        // This command never finishes on its own; it runs until interrupted.
        return false;
    }
}
