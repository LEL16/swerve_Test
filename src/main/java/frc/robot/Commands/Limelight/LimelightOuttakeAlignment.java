package frc.robot.Commands.Limelight;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

public class LimelightOuttakeAlignment extends Command {
    private final LimelightSubsystem m_limelightSubsystem;
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final PIDController m_angleToSpeakerController;

    private double m_distanceToTag;
    private double m_calculatedAngle;
    private double m_calculatedAngleExperimental;

    public LimelightOuttakeAlignment(LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) {
        m_limelightSubsystem = limelightSubsystem;
        m_outtakeSubsystem = outtakeSubsystem;

        m_angleToSpeakerController = new PIDController(1, 0, 0.0);

        addRequirements(m_outtakeSubsystem);
    }


    @Override
    public void execute() {
        m_distanceToTag = m_limelightSubsystem.getDistance("Area");
        m_calculatedAngle = m_distanceToTag * -8.78492 + 57.4318;
        m_calculatedAngleExperimental = Math.toDegrees(Math.atan(Units.inchesToMeters(77) / m_distanceToTag));
        // 1.32 m to tag for Trig

        m_outtakeSubsystem.rotate(m_angleToSpeakerController.calculate(m_outtakeSubsystem.getAngle(), m_calculatedAngleExperimental));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_outtakeSubsystem.rotate(0);
        m_outtakeSubsystem.outtake(0);
    }
}
