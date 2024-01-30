package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightPathfindingCommand extends Command {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private LimelightSubsystem m_limelightSubsystem;

    private int m_tagID;
    private Pose2d m_targetPose2d;

    private GenericEntry tagIdEntry;
    private GenericEntry pose2dEntry;
    public LimelightPathfindingCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem, int tagID) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        m_tagID = tagID;
        m_targetPose2d = m_limelightSubsystem.getTagPose2d(m_tagID);

        ShuffleboardLayout limelightPathfindingLayout = Shuffleboard.getTab("Limelight").getLayout("Pathfinding", BuiltInLayouts.kList).withSize(2, 2);
        tagIdEntry = limelightPathfindingLayout.add("Tag ID", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        // pose2dEntry = limelightPathfindingLayout.add("Pose2d Target", new Pose2d(0, 0, new Rotation2d(0))).withWidget(BuiltInWidgets.kTextView).getEntry();

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        AutoBuilder.pathfindToPose(m_targetPose2d, new PathConstraints(2.0, 2.0, Units.degreesToRadians(180), Units.degreesToRadians(180)), 0.0, 0.0); // Pathfind to the target pose

        while (!m_limelightSubsystem.getTagFound()) { m_drivetrainSubsystem.drive(0, 0, 2.5, false); } // Spin until the tag is found
        if (m_limelightSubsystem.getTagFound()) { new LimelightAlignmentCommand(m_drivetrainSubsystem, m_limelightSubsystem); } // Align to the tag

        tagIdEntry.setDouble(m_tagID);
        pose2dEntry.setString(m_targetPose2d.toString());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}
