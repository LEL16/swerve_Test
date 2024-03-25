package frc.robot.Subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable networkTable;

    private NetworkTableEntry m_angleX;

    private final GenericEntry m_drivetrainAngleChangeEntry;

    public LimelightSubsystem() {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_angleX = networkTable.getEntry("tx");

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout limelightLayout = tab.getLayout("Limelight", BuiltInLayouts.kList).withSize(2, 3)
                .withPosition(6, 0);
        m_drivetrainAngleChangeEntry = limelightLayout
                .add("Desired Drivetrain Angle Change", getDrivetrainAngleChange() + " rad").getEntry();
    }

    @Override
    public void periodic() {
        m_drivetrainAngleChangeEntry.setString(getDrivetrainAngleChange() + " rad");
    }

    /**
     * Returns the change in drivetrain angle necessary for shooting based on
     * limelight input.
     * 
     * @return Change in drivetrain angle (rad).
     */
    public double getDrivetrainAngleChange() {
        return -Math.toRadians(m_angleX.getDouble(0));
    }
}