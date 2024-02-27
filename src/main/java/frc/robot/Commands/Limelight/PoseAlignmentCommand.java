package frc.robot.Commands.Limelight;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class PoseAlignmentCommand extends Command {
    DrivetrainSubsystem m_drivetrainSubsystem;

    Supplier<Pose2d> m_robotPose;
    Pose2d m_targetPose;

    double m_angularError;

    PIDController m_rotationPID;

    // GenericEntry m_angularErrorEntry;

    public PoseAlignmentCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> robotPose,
            Pose2d targetPose) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        m_robotPose = robotPose;
        m_targetPose = targetPose;

        m_rotationPID = new PIDController(0.5, 0, 0);

        // m_angularErrorEntry = Shuffleboard.getTab("Limelight").add("Angular Error", m_angularError).getEntry();

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_angularError = Math.atan2(m_targetPose.getTranslation().getY() - m_robotPose.get().getTranslation().getY(),
                m_targetPose.getTranslation().getX() - m_robotPose.get().getTranslation().getX())
                - m_robotPose.get().getRotation().getRadians();
        
        m_drivetrainSubsystem.drive(0, 0, m_rotationPID.calculate(m_angularError), false);
        // m_angularErrorEntry.setDouble(m_angularError);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_angularError) < 0.1;
    }

}
