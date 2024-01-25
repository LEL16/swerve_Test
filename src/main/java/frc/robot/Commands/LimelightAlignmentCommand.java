package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightAlignmentCommand extends Command {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private PIDController xPidController;
    private PIDController yPidController;
    private PIDController rotPidController;

    private double xVelocity;
    private double yVelocity;
    private double angVelocity;

    private final double kDistanceToPlayer = 0.80;

    private final double kDeadband = 0.05;
    private final double kTolerance = 0.05;

    private final String kTrackingMode; // "translational" = translational tracking, rotational" = rotational tracking

    public LimelightAlignmentCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem,
            String trackingMode) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        kTrackingMode = trackingMode;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        xPidController = new PIDController(0.5, 0, 0);
        yPidController = new PIDController(0.5, 0, 0);
        rotPidController = new PIDController(0.5, 0, 0);
    }

    @Override
    public void execute() {
        xVelocity = xPidController
                .calculate(kTrackingMode == "translational" ? m_limelightSubsystem.getXTargetAngle() : 0);
        angVelocity = rotPidController
                .calculate(kTrackingMode == "rotational" ? m_limelightSubsystem.getYTargetAngle() : 0);
        yVelocity = -yPidController.calculate((m_limelightSubsystem.getDistance() -
                kDistanceToPlayer) * 100);

        xVelocity = (Math.abs(xVelocity) < kDeadband) ? 0 : xVelocity;
        angVelocity = (Math.abs(angVelocity) < kDeadband) ? 0 : angVelocity;
        yVelocity = (Math.abs(yVelocity) < kDeadband) ? 0 : yVelocity;

        m_drivetrainSubsystem.drive(
                yVelocity,
                xVelocity,
                angVelocity,
                (kTrackingMode == "translational") ? true : false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_limelightSubsystem.getXTargetAngle()) <= kTolerance
                && Math.abs(m_limelightSubsystem.getDistance() - kDistanceToPlayer) <= kTolerance / 5;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}