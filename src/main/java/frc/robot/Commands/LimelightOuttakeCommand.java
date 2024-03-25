package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

public class LimelightOuttakeCommand extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final OuttakeSubsystem m_outtakeSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;

    private final double m_outtakeRateSupplier;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    /**
     * Command to engage the outtake autonomously with limelight input. Aims autonomously.
     * 
     * @param drivetrainSubsystem The drivetrain subsystem.
     * @param outtakeSubsystem The outtake subsystem.
     * @param limelightSubsystem The limelight subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     * @param maxTime The maximum time alloted for this command (ms).
     */
    public LimelightOuttakeCommand(DrivetrainSubsystem drivetrainSubsystem, OuttakeSubsystem outtakeSubsystem, LimelightSubsystem limelightSubsystem, double outtakeRateSupplier, long maxTime) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_limelightSubsystem = limelightSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;

        this.m_maxTime = maxTime;
        this.m_isTimeRecorded = false;

        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_outtakeSubsystem.outtake(m_outtakeRateSupplier);
        m_outtakeSubsystem.rotate((m_drivetrainSubsystem.calculateOuttakeAngle() - m_outtakeSubsystem.getAngle()) * 10);
    }

    @Override
    public boolean isFinished() { return System.currentTimeMillis() > m_recordedTime + m_maxTime; }

    @Override
    public void end(boolean interrupted) { 
        m_outtakeSubsystem.outtake(0);
        m_outtakeSubsystem.rotate(0);
        m_isTimeRecorded = false;
    }
}