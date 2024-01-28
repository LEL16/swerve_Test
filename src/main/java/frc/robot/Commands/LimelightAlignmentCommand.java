package frc.robot.Commands;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightAlignmentCommand extends Command {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private PIDController m_xPID; // PID controller for the x-axis
    private PIDController m_yPID; // PID controller for the y-axis
    private PIDController m_rotPID; // PID controller for the rotation

    private double m_xVel; // Velocity of the robot in the x-axis
    private double m_yVel; // Velocity of the robot in the y-axis
    private double m_rotVel; // Velocity of the robot's rotation

    private String m_trackingMode; // Mode of tracking (translational or rotational)
    private double m_distanceToTag; // Distance from the robot to the player

    GenericEntry xVelEntry;
    GenericEntry yVelEntry;
    GenericEntry rotVelEntry;
    GenericEntry trackingModeEntry;
    GenericEntry powerLimit;

    public LimelightAlignmentCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        m_xPID = new PIDController(0.5, 0, 0);
        m_yPID = new PIDController(0.5, 0, 0);
        m_rotPID = new PIDController(0.5, 0, 0);

        ShuffleboardLayout tagTrackingLayout = Shuffleboard.getTab("Limelight").getLayout("Tag Tracking Data", BuiltInLayouts.kList).withSize(2, 2);

        xVelEntry = tagTrackingLayout.add("X Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        yVelEntry = tagTrackingLayout.add("Y Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        rotVelEntry = tagTrackingLayout.add("Rotational Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        trackingModeEntry = tagTrackingLayout.add("Tracking Mode", "Translational").withWidget(BuiltInWidgets.kTextView).getEntry();

        powerLimit = Shuffleboard.getTab("Limelight").add("Power Limit", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_xVel = MathUtil.applyDeadband(m_xPID.calculate(m_limelightSubsystem.getXTargetAngle()), 0.05) * powerLimit.getDouble(1); // Calculate the velocity of the robot in the x-axis
        m_yVel = MathUtil.applyDeadband(m_yPID.calculate(m_limelightSubsystem.getDistance("Area") - m_distanceToTag), 0.05) * 2.5 * powerLimit.getDouble(1); // Calculate the velocity of the robot in the y-axis
        m_rotVel = MathUtil.applyDeadband(m_rotPID.calculate(m_limelightSubsystem.getXTargetAngle()), 0.5) * powerLimit.getDouble(1); // Calculate the velocity of the robot's rotation

        m_trackingMode = m_limelightSubsystem.getTrackingMode();
        m_distanceToTag = m_limelightSubsystem.getDistanceToTag();

        m_drivetrainSubsystem.drive(
                m_trackingMode == "translational" ? m_xVel : 0,
                m_trackingMode == "translational" ? m_yVel : 0,
                m_trackingMode == "rotational" ? m_rotVel : 0,
                m_trackingMode == "translational" ? true : false); // Drives the robot based on the tracking mode selected

        xVelEntry.setDouble(m_xVel); // Shuffleboard data
        yVelEntry.setDouble(m_yVel);
        rotVelEntry.setDouble(m_rotVel);
        trackingModeEntry.setString(m_trackingMode);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_limelightSubsystem.getXTargetAngle()) <= 0.5 && (Math.abs(m_limelightSubsystem.getDistance("Area") - m_distanceToTag) <= 0.05 || m_trackingMode == "rotational"); // If the robot is within 0.5 degrees of the target and 0.05 meters of the target, the command is finished
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}