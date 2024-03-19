package frc.robot.Commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Utils.AllianceUtil;

public class LimelightAutoShoot extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final OuttakeSubsystem m_outtakeSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;

    private Pose2d m_speakerPose;
    private Debouncer m_setpointDebouncer;

    public LimelightAutoShoot(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem,
            OuttakeSubsystem outtakeSubsystem, LimelightSubsystem limelightSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_limelightSubsystem = limelightSubsystem;

        addRequirements(m_intakeSubsystem, m_outtakeSubsystem);
    }

    @Override
    public void initialize() {
        m_speakerPose = AllianceUtil.getSpeakerPose("blue");
    }

    @Override
    public void execute() {
        double distance = Math.hypot(m_speakerPose.getTranslation().getX() - m_drivetrainSubsystem.getPosition().getX(),
                m_speakerPose.getTranslation().getY() - m_drivetrainSubsystem.getPosition().getY());

        double angle = m_limelightSubsystem.getShooterAngle(distance);
        Rotation2d desiredAngle = Rotation2d.fromRadians(angle);

        m_outtakeSubsystem.setDesiredAngle(desiredAngle);

        // double motorRPM = m_limelightSubsystem.getShooterRPM(distanc%e);
        // m_outtakeSubsystem.outtake(motorRPM);

        Rotation2d armAngleError = desiredAngle.minus(Rotation2d.fromRadians(m_outtakeSubsystem.getAngle()));
        // double shooterError = motorRPM - m_outtakeSubsystem.getSpeed();

        if (m_setpointDebouncer.calculate(
                Math.abs(armAngleError.getRadians()) < Math.toRadians(1))) {
            // && shooterError < 800)) {
            m_intakeSubsystem.intake(200);
        }

        SmartDashboard.putNumber("Auto Shoot/Desired Angle", desiredAngle.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intake(0);
        m_outtakeSubsystem.rotate(0);
        m_outtakeSubsystem.outtake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
