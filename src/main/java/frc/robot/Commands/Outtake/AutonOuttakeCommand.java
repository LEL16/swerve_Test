package frc.robot.Commands.Outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class AutonOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final double m_outtakeRatePercentSupplier;
    private final double m_outtakeAngle;

    private final double m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    private final PIDController m_anglePIDController;

    /**
     * Command to engage the outtake using joystick input.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRatePercentSupplier The desired outtake rate (percent). + for outtake, - for intake.
     * @param outtakeAngle The desired outtake angle (rad).
     * @param maxTime The maximum time alloted for this command (sec).
     */
    public AutonOuttakeCommand(OuttakeSubsystem outtakeSubsystem, double outtakeRatePercentSupplier, double outtakeAngle, double maxTime) {
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_outtakeRatePercentSupplier = outtakeRatePercentSupplier;
        this.m_outtakeAngle = outtakeAngle;

        this.m_maxTime = maxTime;
        this.m_isTimeRecorded = false;

        m_anglePIDController = new PIDController(1, 0, 0.0);

        addRequirements(outtakeSubsystem);
    }

    /**
     * Command to engage the outtake using joystick input. Keeps outtake angle the same.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRatePercentSupplier The desired outtake rate (percent). + for outtake, - for intake.
     * @param maxTime The maximum time alloted for this command (sec).
     */
    public AutonOuttakeCommand(OuttakeSubsystem outtakeSubsystem, double outtakeRatePercentSupplier, double maxTime) {
        this(outtakeSubsystem, outtakeRatePercentSupplier, outtakeSubsystem.getAngle(), maxTime);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_outtakeSubsystem.outtake(-m_outtakeRatePercentSupplier / 100 * OuttakeSubsystem.kOuttakeMaxRate);
        m_outtakeSubsystem.rotate(m_anglePIDController.calculate(m_outtakeSubsystem.getAngle(), m_outtakeAngle));
    }

    @Override
    public boolean isFinished() { return System.currentTimeMillis() > m_recordedTime + m_maxTime * 1000; }

    @Override
    public void end(boolean interrupted) { 
        m_outtakeSubsystem.outtake(0);
        m_outtakeSubsystem.rotate(0);
    }
}