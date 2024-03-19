package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Utils.AllianceUtil;

public class LimelightOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_outtakeRateSupplier;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    private final PIDController m_anglePIDController;

    private Pose2d m_speakerPose;
    private Debouncer m_setpointDebouncer;

    /**
     * Command to engage the outtake autonomously with limelight input. Aims
     * autonomously.
     * 
     * @param outtakeSubsystem    The outtake subsystem.
     * @param limelightSubsystem  The limelight subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     * @param maxTime             The maximum time alloted for this command (ms).
     */
    public LimelightOuttakeCommand(OuttakeSubsystem outtakeSubsystem, LimelightSubsystem limelightSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, double outtakeRateSupplier, long maxTime) {
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_limelightSubsystem = limelightSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        this.m_maxTime = maxTime;
        this.m_isTimeRecorded = false;

        m_anglePIDController = new PIDController(10.0, 0, 0);

        addRequirements(outtakeSubsystem);
    }

    public void initialize() {
        m_speakerPose = AllianceUtil.getSpeakerPose("blue");
    }

    @Override
    public void execute() {
        double distance = Math.hypot(Math.abs(m_drivetrainSubsystem.getPosition().getX() - m_speakerPose.getTranslation().getX()),
                Math.abs(m_drivetrainSubsystem.getPosition().getY() - m_speakerPose.getTranslation().getY()));

        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_outtakeSubsystem.outtake(m_outtakeRateSupplier);
        m_outtakeSubsystem.rotate(m_anglePIDController.calculate(m_outtakeSubsystem.getAngle(),
                m_limelightSubsystem.getShooterAngle(distance)));

        SmartDashboard.putNumber("Distance (Pose)", distance);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > m_recordedTime + m_maxTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_outtakeSubsystem.outtake(0);
        m_outtakeSubsystem.rotate(0);
        m_isTimeRecorded = false;
    }
}