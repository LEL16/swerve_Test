package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OuttakeSubsystem extends SubsystemBase {
    private static final double kOuttakeGearRatio = (1.0 / 3.0);
    public static final double kOuttakeMaxRate = 5676.0 * kOuttakeGearRatio; // RPM

    private final CANSparkMax m_frontShooterMotor;
    private final CANSparkMax m_backShooterMotor;
    private final CANSparkMax m_linearActuatorMotor;

    private final RelativeEncoder m_shooterEncoder;
    private final RelativeEncoder m_linearActuatorEncoder;
    private final SparkAbsoluteEncoder m_linearActuatorAbsoluteEncoder;

    private final SimpleMotorFeedforward m_outtakeFeedforward = new SimpleMotorFeedforward(0, 0.00634);
    private final SimpleMotorFeedforward m_linearActuatorFeedforward = new SimpleMotorFeedforward(0, 0.00815);

    private final GenericEntry m_outtakeRateEntry;
    private final GenericEntry m_linearActuatorPositionEntry;
    private final GenericEntry m_linearActuatorSpeedEntry;

    private double m_outtakeRate;
    private double m_actuatorRate;

    public OuttakeSubsystem() {
        m_frontShooterMotor = new CANSparkMax(Constants.OUTTAKE_MOTOR_1, MotorType.kBrushless);
        m_backShooterMotor = new CANSparkMax(Constants.OUTTAKE_MOTOR_2, MotorType.kBrushless);
        m_linearActuatorMotor = new CANSparkMax(Constants.LINEAR_ACTUATOR_MOTOR, MotorType.kBrushless);

        m_frontShooterMotor.restoreFactoryDefaults();
        m_backShooterMotor.restoreFactoryDefaults();
        m_linearActuatorMotor.restoreFactoryDefaults();

        m_frontShooterMotor.setIdleMode(IdleMode.kBrake);
        m_backShooterMotor.setIdleMode(IdleMode.kBrake);
        m_linearActuatorMotor.setIdleMode(IdleMode.kBrake);

        m_frontShooterMotor.setSmartCurrentLimit(30);
        m_backShooterMotor.setSmartCurrentLimit(30);
        m_linearActuatorMotor.setSmartCurrentLimit(30);

        m_frontShooterMotor.setInverted(false);
        m_backShooterMotor.follow(m_frontShooterMotor, false);
        m_linearActuatorMotor.setInverted(false);

        m_shooterEncoder = m_frontShooterMotor.getEncoder();
        m_linearActuatorEncoder = m_linearActuatorMotor.getEncoder();
        m_linearActuatorAbsoluteEncoder = m_linearActuatorMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        m_shooterEncoder.setPositionConversionFactor(kOuttakeGearRatio);
        m_shooterEncoder.setVelocityConversionFactor(kOuttakeGearRatio);

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout outtakeLayout = tab.getLayout("Outtake", BuiltInLayouts.kList).withSize(2, 1).withPosition(2, 0);
        m_outtakeRateEntry = outtakeLayout.add("Outtake Rate", m_shooterEncoder.getVelocity()).getEntry();
        m_linearActuatorPositionEntry = outtakeLayout.add("Actuator Position", m_linearActuatorAbsoluteEncoder.getPosition()).getEntry();
        m_linearActuatorSpeedEntry = outtakeLayout.add("Speed", m_linearActuatorAbsoluteEncoder.getVelocity()).getEntry();
    }

    /**
     * Engages the outtake.
     * 
     * @param outtakeRate The rate of outtake (rpm).
     */
    public void outtake(double outtakeRate) {
        m_outtakeRate = outtakeRate;
    }

    public void actuate(double actuateRate) {
        m_actuatorRate = actuateRate;
    }

    /** Displays the periodically updated outtake rate on the Shuffleboard */
    public void updateShuffleboard() {
        m_outtakeRateEntry.setDouble(m_shooterEncoder.getVelocity());
        m_linearActuatorPositionEntry.setDouble(m_linearActuatorAbsoluteEncoder.getPosition());
        m_linearActuatorSpeedEntry.setDouble(m_linearActuatorAbsoluteEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        m_frontShooterMotor.setVoltage(m_outtakeFeedforward.calculate(m_outtakeRate));
        m_linearActuatorMotor.setVoltage(m_linearActuatorFeedforward.calculate(m_actuatorRate));
        updateShuffleboard();
    }
}
