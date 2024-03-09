package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private final double kLimelightLensHeight = Constants.LIMELIGHT_LENS_HEIGHT;
    private final double kLimelightAngle = Constants.LIMELIGHT_ANGLE;

    private NetworkTable m_networkTable;

    private NetworkTableEntry m_pipelineId;

    private NetworkTableEntry m_tx;
    private NetworkTableEntry m_ty;
    private NetworkTableEntry m_ta;
    private NetworkTableEntry m_tv;
    private NetworkTableEntry m_tid;
    private NetworkTableEntry m_camMode;
    private NetworkTableEntry m_ledMode;

    private double m_areaDistance;
    private double m_trigDistance;

    private final double[] m_targetHeight = new double[] { 53.0, 53.0, 56.375, 56.375, 52.625, 52.625, 56.375, 56.375, 53.0, 53.0, 52.0, 52.0, 52.0, 52.0, 52.0, 52.0 }; // Inches
    private Pose2d[] m_targetPose2d = new Pose2d[] { new Pose2d(15.23, 0.88, new Rotation2d(0.0)), new Pose2d(15.91, 1.25, new Rotation2d(0.0)), new Pose2d(16.50, 5.00, new Rotation2d(0.0)), new Pose2d(16.30, 5.60, new Rotation2d(0.0)), new Pose2d(14.70, 8.10, new Rotation2d(0.0)), new Pose2d(1.80, 8.20, new Rotation2d(0.0)), new Pose2d(0.65, 5.50, new Rotation2d(0.0)), new Pose2d(0.00, 5.00, new Rotation2d(0.0)), new Pose2d(0.65, 0.70, new Rotation2d(0.0)), new Pose2d(1.20, 0.40, new Rotation2d(0.0)), new Pose2d(12.00, 3.75, new Rotation2d(0.0)), new Pose2d(12.00, 4.50, new Rotation2d(0.0)), new Pose2d(11.25, 4.00, new Rotation2d(0.0)), new Pose2d(5.35, 4.00, new Rotation2d(0.0)), new Pose2d(4.65, 4.50, new Rotation2d(0.0)), new Pose2d(4.65, 3.75, new Rotation2d(0.0)) };

    private GenericEntry pipelineIdEntry;
    private GenericEntry XEntry;
    private GenericEntry YEntry;
    private GenericEntry targetAreaEntry;
    private GenericEntry foundTagEntry;
    private GenericEntry tagIdEntry;
    private GenericEntry areaDistanceEntry;
    private GenericEntry trigDistanceEntry;

    public LimelightSubsystem() {
        m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");

        m_pipelineId = m_networkTable.getEntry("getpipe"); // Pipeline (0-9)
        m_camMode = m_networkTable.getEntry("camMode"); // Vision Processing Mode (0) & Driver Camera Mode (1)
        m_ledMode = m_networkTable.getEntry("ledMode"); // Pipeline default (0), Off (1), Blink (2), & On (3)
        m_tx = m_networkTable.getEntry("tx"); // (-29.8 to 29.8 degrees)
        m_ty = m_networkTable.getEntry("ty"); // (-24.85 to 24.85 degrees)
        m_ta = m_networkTable.getEntry("ta"); // (0% of image to 100% of image)
        m_tv = m_networkTable.getEntry("tv"); // (0 or 1 (found))
        m_tid = m_networkTable.getEntry("tid"); // Tag ID (Integer 0-9)

        ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
        ShuffleboardLayout limelightDataLayout = limelightTab.getLayout("Limelight Data", BuiltInLayouts.kList).withSize(2, 6).withPosition(0, 0);
        
        pipelineIdEntry = limelightDataLayout.add("Pipeline ID", m_pipelineId.getDouble(-1)).getEntry();
        XEntry = limelightDataLayout.add("X Angle", Math.toRadians(m_tx.getDouble(0)) + "rad").getEntry();
        YEntry = limelightDataLayout.add("Y Angle", Math.toRadians(m_ty.getDouble(0)) + "rad").getEntry();
        targetAreaEntry = limelightDataLayout.add("Target Area", m_ta.getDouble(0) + "%").getEntry();
        foundTagEntry = limelightDataLayout.add("Found Tag", (m_tv.getDouble(0) == 1)).getEntry();
        tagIdEntry = limelightDataLayout.add("Tag ID", "ID: " + m_tid.getDouble(0)).getEntry();
        areaDistanceEntry = limelightDataLayout.add("Area Distance", this.getDistance("Area") + "m").getEntry();
        trigDistanceEntry = limelightDataLayout.add("Trig Distance", this.getDistance("Trig") + "m").getEntry();
    }

    @Override
    public void periodic() {
        m_areaDistance = Units.inchesToMeters(54.4 * Math.pow(m_ta.getDouble(0), -0.475)); // Area Distance
        m_trigDistance = Units.inchesToMeters(51.96 - kLimelightLensHeight) / Math.tan(Math.toRadians(m_ty.getDouble(0.0) + kLimelightAngle)); // Trigonometry Distance

        updateShuffleboard();
    }

    /** Sets the pipeline.
     * 
     * @param pipeline The pipeline (0-9).
     */
    public void setPipeline(int pipeline) {
        m_pipelineId.setDouble(pipeline);
    }

    /** Sets the camera mode.
     * 
     * @param mode The camera mode (vision or driver).
     */
    public void setCamMode(String mode) {
        m_camMode.setDouble((mode == "vision") ? 0 : 1);
    }

    /** Sets the LED mode.
     * 
     * @param mode The LED mode (off, blink, on, or default).
     */
    public void setLedMode(String mode) {
        m_ledMode.setDouble((mode == "off") ? 1 : (mode == "blink") ? 2 : (mode == "on") ? 3 : 0);
    }

    /** Returns the distance to the target based on the mode.
     * 
     * @param mode The mode to calculate the distance (Area or Trigonometry).
     * @return The distance to the target (m).
     */
    public double getDistance(String mode) {
        if (mode.equals("Area")) { return m_areaDistance; } 
        else if (mode.equals("Trig") || mode.equals("Trigonometry")) { return m_trigDistance; } 
        else { return 0.0; }
    }

    /** Returns the X offset.
     * 
     * @return The X offset (rad).
     */
    public double getXOffset() {
        return Math.toRadians(m_tx.getDouble(0));
    }

    /** Returns the Y offset.
     * 
     * @return The Y offset (rad).
     */
    public double getYOffset() {
        return Math.toRadians(m_ty.getDouble(0));
    }

    /** Returns the target status.
     * 
     * @return The target status (true or false).
     */
    public boolean getTargetStatus() {
        return m_tv.getDouble(0) != 0;
    }

    /** Returns the target ID.
     * 
     * @return The target ID (1-16).
     */
    public double getTargetIdentity() {
        return m_tid.getDouble(0.0);
    }

    /** Returns the target pose based on its ID.
     * 
     * @param targetId The target ID (1-16).
     * @return The target Pose2d.
     */
    public Pose2d getTargetPose(int targetId) {
        if (targetId < 1 || targetId > 16) { return new Pose2d(); }
        return m_targetPose2d[targetId - 1];
    }

    /** Returns the target height based on its ID.
     * 
     * @param targetId The target ID (1-16).
     * @return The target height (inches).
     */
    public double getTargetHeight(int targetId) {
        if (targetId < 1 || targetId > 16) { return 0.0; }
        return m_targetHeight[targetId - 1];
    }

    /** Returns the calculated shooter angle based on trigonometric distance. 
     * 
     * @return The calculated shooter angle (rad).
    */
    public double getShooterAngle() {
        return -0.190354293083 * getDistance("Trig") - 1.930509;
    }

    /** Displays the periodically updated Limelight values on Shuffleboard. */
     private void updateShuffleboard() {
        pipelineIdEntry.setDouble(m_pipelineId.getDouble(-1));
        XEntry.setString(Math.toRadians(m_tx.getDouble(0)) + "rad");
        YEntry.setString(Math.toRadians(m_ty.getDouble(0)) + "rad");
        targetAreaEntry.setDouble(m_ta.getDouble(0));
        foundTagEntry.setBoolean(m_tv.getDouble(0) == 1);
        tagIdEntry.setString("ID: " + m_tid.getDouble(0));
        areaDistanceEntry.setString(this.getDistance("Area") + "m");
        trigDistanceEntry.setString(this.getDistance("Trig") + "m");
    }
}