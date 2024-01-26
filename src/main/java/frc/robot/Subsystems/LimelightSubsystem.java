package frc.robot.Subsystems;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class LimelightSubsystem extends SubsystemBase {
    private final double kPipelineId;
    private final boolean kCameraMode = true, kLedMode = true;
    private final double kXResolution = 320, kYResolution = 240; // px

    private final double kLimelightLensHeight = Units.inchesToMeters(1.75);
    private final double kLimelightAngle = 10; // degrees (upwards)

    private double XTargetAngle, YTargetAngle, targetArea, foundTag, targetAreaDistance, trigDistance;
    private boolean foundTagBool;

    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry pipelineEntry, camModeEntry, ledModeEntry, XEntry, YEntry,
            targetAreaEntry, foundTagEntry;

    private GenericEntry txEntry;
    private GenericEntry tyEntry;
    private GenericEntry taEntry;

    public LimelightSubsystem(double pipelineId) {
        kPipelineId = pipelineId;

        ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
        txEntry = limelightTab.add("tx", 0).getEntry();
        tyEntry = limelightTab.add("ty", 0).getEntry();
        taEntry = limelightTab.add("ta", 0).getEntry();
    }


    @Override
    public void periodic() {
        pipelineEntry = networkTable.getEntry("pipeline");
        pipelineEntry.setNumber(kPipelineId);

        camModeEntry = networkTable.getEntry("camMode");
        camModeEntry.setNumber(kCameraMode ? 1 : 0); // 1 = on, 0 = off

        ledModeEntry = networkTable.getEntry("ledMode");
        ledModeEntry.setNumber(kLedMode ? 3 : 1); // 3 = on, 1 = off

        XEntry = networkTable.getEntry("tx"); // degrees horizontal
        YEntry = networkTable.getEntry("ty"); // degrees vertical
        targetAreaEntry = networkTable.getEntry("ta"); // area in percentage
        foundTagEntry = networkTable.getEntry("tv"); // not equal to 0 for tag found

        XTargetAngle = XEntry.getDouble(0.0);
        YTargetAngle = YEntry.getDouble(0.0);
        targetArea = targetAreaEntry.getDouble(0.0);
        foundTag = foundTagEntry.getDouble(0.0);
        foundTagBool = foundTag != 0;

        txEntry.setDouble(XTargetAngle);
        tyEntry.setDouble(YTargetAngle);
        taEntry.setDouble(targetArea);    
    }

    public double getXTargetAngle() {
        return XTargetAngle;
    }

    public double getYTargetAngle() {
        return YTargetAngle;
    }

    public double getDistance() {
        targetAreaDistance = Units.inchesToMeters(54.4 * Math.pow(targetArea, -0.475)); // calculate curve using area
                                                                                        // (calculated curve)
        trigDistance = Units.inchesToMeters((50.13 - kLimelightLensHeight)
                / Math.tan(Math.toRadians(YTargetAngle + kLimelightAngle))); // use when mounted "d = (h2-h1) / tan(a1+a2)"

        return targetAreaDistance;
    }
}

