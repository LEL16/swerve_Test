package frc.robot.Subsystems;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LimelightSubsystem extends SubsystemBase {
    private final double kLimelightLensHeight = Units.inchesToMeters(1.75);
    private final double kLimelightAngle = 10.0;

    private NetworkTable m_networkTable;

    private NetworkTableEntry m_pipelineId;
    private NetworkTableEntry m_camMode;
    private NetworkTableEntry m_ledMode;

    private NetworkTableEntry m_tx;
    private NetworkTableEntry m_ty;
    private NetworkTableEntry m_ta;
    private NetworkTableEntry m_tv;
    private double[] m_tid;

    private double[] m_botPose;
    private double[] m_targetPose;

    private double m_areaDistance;
    private double m_trigDistance;
    private double[] m_tagHeight = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0 }; // Add heights here? Or anyway you prefer.

    private GenericEntry pipelineIdEntry;
    private GenericEntry camModeEntry;
    private GenericEntry ledModeEntry;
    private GenericEntry XEntry;
    private GenericEntry YEntry;
    private GenericEntry targetAreaEntry;
    private GenericEntry foundTagEntry;
    private GenericEntry tagIdEntry;
    private GenericEntry botPoseEntry;
    private GenericEntry targetPoseEntry;
    private GenericEntry areaDistanceEntry;
    private GenericEntry trigDistanceEntry;

    public LimelightSubsystem() {
        m_networkTable = NetworkTableInstance.getDefault().getTable("limelight"); 

        m_pipelineId = m_networkTable.getEntry("getpipe"); // Active pipeline index of the camera (0-9)
        
        m_camMode = m_networkTable.getEntry("camMode"); // Vision processing mode (0) or driver camera mode (1) (increases exposure, disables vision processing)
        m_ledMode = m_networkTable.getEntry("ledMode"); // Current LED mode of the camera (0-pipeline default, 1-off, 2-blink, 3-on)

        m_tx = m_networkTable.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
        m_ty = m_networkTable.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
        m_ta = m_networkTable.getEntry("ta"); // Target area (0% of image to 100% of image)
        m_tv = m_networkTable.getEntry("tv"); // Any valid targets (0 or 1 (found))
        m_tid = m_networkTable.getEntry("tid").getDoubleArray(new double[6]); // Target april tag ID (0, 1, 2, 3, 4, 5)

        // Needs testing!
        m_botPose = m_networkTable.getEntry("botpose").getDoubleArray(new double[6]); // Pose of the robot in the world frame (x, y, z, pitch, yaw, roll)
        m_targetPose = m_networkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); // Pose of the target in the camera frame (x, y, z, pitch, yaw, roll)

        ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
        
        pipelineIdEntry = limelightTab.add("Pipeline ID", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 9)).getEntry();
        camModeEntry = limelightTab.add("Camera Mode", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        ledModeEntry = limelightTab.add("LED Mode", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        ShuffleboardLayout limelightDataLayout = Shuffleboard.getTab("Limelight").getLayout("Limelight Data", BuiltInLayouts.kList).withSize(2, 3);
        XEntry = limelightDataLayout.add("X Angle", 0).getEntry();
        YEntry = limelightDataLayout.add("Y Angle", 0).getEntry();
        targetAreaEntry = limelightDataLayout.add("Target Area", 0).getEntry();
        foundTagEntry = limelightDataLayout.add("Found Tag", 0).getEntry();
        tagIdEntry = limelightDataLayout.add("Tag ID", 0).getEntry();
        botPoseEntry = limelightDataLayout.add("Bot Pose", 0).getEntry();
        targetPoseEntry = limelightDataLayout.add("Target Pose", 0).getEntry();
        areaDistanceEntry = limelightDataLayout.add("Area Distance", 0).getEntry();
        trigDistanceEntry = limelightDataLayout.add("Trig Distance", 0).getEntry();
    }

    @Override
    public void periodic() {
        m_areaDistance = Units.inchesToMeters(54.4 * Math.pow(m_ta.getDouble(0), -0.475)); // Calculates distance based on graphed ta values, used google sheets to calculate curve
        // FIXME: Hriday's making the array of heights, so this is just a placeholder. Refer to m_tagHeight array above, or implement your own dictionary!
        m_trigDistance = Units.inchesToMeters(m_tagHeight[(int) m_tid[0]] - kLimelightLensHeight) / Math.tan(Math.toRadians(m_ty.getDouble(0.0) + kLimelightAngle)); // Calculates distance using trigonometry. Reference a triangle and the notion that tan(theta) = opposite/adjacent. Opposite = height of target - height of camera, adjacent = distance from camera to target, theta = angle of camera to target. Rearrange to get d = (h2-h1) / tan(a1+a2)

        m_pipelineId.setNumber(pipelineIdEntry.getDouble(0));
        m_camMode.setNumber(camModeEntry.getBoolean(false) ? 1 : 0);
        m_ledMode.setNumber(ledModeEntry.getBoolean(false) ? 3 : 1);

        XEntry.setDouble(m_tx.getDouble(0));
        YEntry.setDouble(m_ty.getDouble(0));
        targetAreaEntry.setDouble(m_ta.getDouble(0));
        foundTagEntry.setDouble(m_tv.getDouble(0));
        tagIdEntry.setDouble(m_tid[0]);
        botPoseEntry.setDoubleArray(m_botPose);
        targetPoseEntry.setDoubleArray(m_targetPose);
        areaDistanceEntry.setDouble(m_areaDistance);
        trigDistanceEntry.setDouble(m_trigDistance);
    }

    public double getAreaDistance(String mode) {
        if (mode.equals("area")) { return m_areaDistance; } 
        else if (mode.equals("trig")) { return m_trigDistance; } 
        else { return 0.0; }
    }
}