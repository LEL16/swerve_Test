// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  private static final double kTrackWidth = 0.60; // meters

  public static final double kMaxSpeed = (5676.0 / 60.0) * SwerveModule.kDriveGearRatio * SwerveModule.kWheelRadius * 2
      * Math.PI; // meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kTrackWidth / 2.0); // Radians
                                                                                                              // per sec

  private static final Translation2d m_frontLeftLocation = new Translation2d(kTrackWidth / 2.0, kTrackWidth / 2.0);
  private static final Translation2d m_frontRightLocation = new Translation2d(kTrackWidth / 2.0, -kTrackWidth / 2.0);
  private static final Translation2d m_backLeftLocation = new Translation2d(-kTrackWidth / 2.0, kTrackWidth / 2.0);
  private static final Translation2d m_backRightLocation = new Translation2d(-kTrackWidth / 2.0, -kTrackWidth / 2.0);

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveDriveKinematics m_kinematics;

  private final SwerveDriveOdometry m_odometry;

  private final GenericEntry m_frontLeftDriveSpeedEntry;
  private final GenericEntry m_frontLeftSteerAngleEntry;
  private final GenericEntry m_frontRightDriveSpeedEntry;
  private final GenericEntry m_frontRightSteerAngleEntry;
  private final GenericEntry m_backLeftDriveSpeedEntry;
  private final GenericEntry m_backLeftSteerAngleEntry;
  private final GenericEntry m_backRightDriveSpeedEntry;
  private final GenericEntry m_backRightSteerAngleEntry;
  private final GenericEntry m_odometryXEntry;
  private final GenericEntry m_odometryYEntry;
  private final GenericEntry m_odometryThetaEntry;

  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rot;
  private boolean m_fieldRelative;

  private Field2d m_field = new Field2d();

  private final MutableMeasure<Voltage> m_appliedVoltage;
  private final MutableMeasure<Distance> m_distance;
  private final MutableMeasure<Velocity<Distance>> m_velocity;

  private final SysIdRoutine m_sysIdRoutine;

  public DrivetrainSubsystem() {
    m_frontLeft = new SwerveModule(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    m_frontRight = new SwerveModule(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
    m_backLeft = new SwerveModule(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET);
    m_backRight = new SwerveModule(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation);

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_navx.getRotation2d(), getModulePositions());

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    ShuffleboardLayout frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(0, 0);
    m_frontLeftDriveSpeedEntry = frontLeftLayout.add("Drive Speed", m_frontLeft.getState().speedMetersPerSecond)
        .getEntry();
    m_frontLeftSteerAngleEntry = frontLeftLayout.add("Steer Angle", m_frontLeft.getState().angle.getDegrees())
        .getEntry();

    ShuffleboardLayout frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(2, 0);
    m_frontRightDriveSpeedEntry = frontRightLayout.add("Drive Speed", m_frontRight.getState().speedMetersPerSecond)
        .getEntry();
    m_frontRightSteerAngleEntry = frontRightLayout.add("Steer Angle", m_frontRight.getState().angle.getDegrees())
        .getEntry();

    ShuffleboardLayout backLeftLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(4, 0);
    m_backLeftDriveSpeedEntry = backLeftLayout.add("Drive Speed", m_backLeft.getState().speedMetersPerSecond)
        .getEntry();
    m_backLeftSteerAngleEntry = backLeftLayout.add("Steer Angle", m_backLeft.getState().angle.getDegrees()).getEntry();

    ShuffleboardLayout backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(6, 0);
    m_backRightDriveSpeedEntry = backRightLayout.add("Drive Speed", m_backRight.getState().speedMetersPerSecond)
        .getEntry();
    m_backRightSteerAngleEntry = backRightLayout.add("Steer Angle", m_backRight.getState().angle.getDegrees())
        .getEntry();

    ShuffleboardLayout odometryLayout = tab.getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 3).withPosition(0,
        2);
    m_odometryXEntry = odometryLayout.add("X Position", getPosition().getX()).getEntry();
    m_odometryYEntry = odometryLayout.add("Y Position", getPosition().getY()).getEntry();
    m_odometryThetaEntry = odometryLayout.add("Angle", getAngle().getDegrees()).getEntry();

    m_field = new Field2d();
    tab.add(m_field);

    AutoBuilder.configureHolonomic(
        () -> new Pose2d(this.getPosition(), this.getAngle()),
        (pose) -> setPose(pose.getX(), pose.getY(), pose.getRotation().getDegrees()),
        () -> new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot),
        (chassisSpeed) -> drive(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond,
            chassisSpeed.omegaRadiansPerSecond, false),
        new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.DRIVE_KP, Constants.DRIVE_KI, Constants.DRIVE_KD), // Translational
            new PIDConstants(Constants.STEER_KP, Constants.STEER_KI, Constants.STEER_KD), // Rotational
            3.81,
            kTrackWidth,
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
          }
          return false;
        },
        this);

    m_appliedVoltage = mutable(Volts.of(0));
    m_distance = mutable(Meters.of(0));
    m_velocity = mutable(MetersPerSecond.of(0));

    m_sysIdRoutine = new SysIdRoutine(
        // Empty congif = 1 volt/second ramp rate and 7 volt step voltage
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Plumb the voltage into the drive motors
            (Measure<Voltage> volts) -> {
              m_frontLeft.getDriveMotor().setVoltage(volts.in(Volts));
              m_frontRight.getDriveMotor().setVoltage(volts.in(Volts));
              m_backLeft.getDriveMotor().setVoltage(volts.in(Volts));
              m_backRight.getDriveMotor().setVoltage(volts.in(Volts));
            },
            // Record frames of data for each motor on the drive mechanism
            log -> {
              // Record a frame for the front-left motor
              log.motor("front-left")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          m_frontLeft.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(m_frontLeft.getDrivePosition().distanceMeters, Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_frontLeft.getState().speedMetersPerSecond, MetersPerSecond));
              // Record a frame for the front-right motor
              log.motor("front-right")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          m_frontRight.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(m_frontRight.getDrivePosition().distanceMeters, Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_frontRight.getState().speedMetersPerSecond, MetersPerSecond));
              // Record a frame for the back-left motor
              log.motor("back-left")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          m_backLeft.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(m_backLeft.getDrivePosition().distanceMeters, Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_backLeft.getState().speedMetersPerSecond, MetersPerSecond));
              // Record a frame for the back-right motor
              log.motor("back-right")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          m_backRight.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(m_backRight.getDrivePosition().distanceMeters, Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_backRight.getState().speedMetersPerSecond, MetersPerSecond));
            },
            // Require this subsystem
            this));
  }

  /**
   * Drives the robot.
   *
   * @param xSpeed        The speed of the robot in the x direction (m/s).
   * @param ySpeed        The speed of the robot in the y direction (m/s).
   * @param rot           The angular rate of the robot (rad/s).
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;
    m_fieldRelative = fieldRelative;
  }

  /**
   * Returns the current odometric position of the robot.
   * 
   * @return The current odometric position of the robot.
   */
  public Translation2d getPosition() {
    return m_odometry.getPoseMeters().getTranslation();
  }

  /**
   * Returns the current odometric angle of the robot.
   * 
   * @return The current odometric angle of the robot.
   */
  public Rotation2d getAngle() {
    return m_odometry.getPoseMeters().getRotation();
  }

  /**
   * Sets the odometric position and angle of the robot.
   *
   * @param xPos  The position of the robot in the x direction (m).
   * @param yPos  The position of the robot in the y direction (m).
   * @param theta The angle of the robot (rad).
   */
  public void setPose(double xPos, double yPos, double theta) {
    m_odometry.resetPosition(m_navx.getRotation2d(), getModulePositions(),
        new Pose2d(xPos, yPos, new Rotation2d(theta)));
  }

  /**
   * Sets all relative turning encoders used in PID to all absolute turning
   * encoder position. Do not call periodically.
   */
  public void alignTurningEncoders() {
    m_frontLeft.alignTurningEncoders();
    m_frontRight.alignTurningEncoders();
    m_backLeft.alignTurningEncoders();
    m_backRight.alignTurningEncoders();
  }

  /**
   * Returns initial positions of the swerve modules as a SwerveModulePosition[].
   * 
   * @return The initial positions of the swerve modules as a
   *         SwerveModulePosition[].
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getDrivePosition(),
        m_frontRight.getDrivePosition(),
        m_backLeft.getDrivePosition(),
        m_backRight.getDrivePosition()
    };
  }


  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /** Displays the periodically updated robot poses on the Shuffleboard */
  public void updateShuffleboard() {
    m_frontLeftDriveSpeedEntry.setDouble(m_frontLeft.getState().speedMetersPerSecond);
    m_frontLeftSteerAngleEntry.setDouble(m_frontLeft.getState().angle.getDegrees());

    m_frontRightDriveSpeedEntry.setDouble(m_frontRight.getState().speedMetersPerSecond);
    m_frontRightSteerAngleEntry.setDouble(m_frontRight.getState().angle.getDegrees());

    m_backLeftDriveSpeedEntry.setDouble(m_backLeft.getState().speedMetersPerSecond);
    m_backLeftSteerAngleEntry.setDouble(m_backLeft.getState().angle.getDegrees());

    m_backRightDriveSpeedEntry.setDouble(m_backRight.getState().speedMetersPerSecond);
    m_backRightSteerAngleEntry.setDouble(m_backRight.getState().angle.getDegrees());

    m_odometryXEntry.setDouble(getPosition().getX());
    m_odometryYEntry.setDouble(getPosition().getY());
    m_odometryThetaEntry.setDouble(getAngle().getDegrees());
  }

  @Override
  public void periodic() {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        m_fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot, getAngle())
            : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    m_odometry.update(
        m_navx.getRotation2d(),
        getModulePositions());

    updateShuffleboard();
  }

  private class SwerveModule {
    private static final double kWheelRadius = 0.050165; // Meters
    private static final double kDriveGearRatio = (16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    private static final double kSteerGearRatio = 7.0 / 150.0;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final CANcoder m_turningCANcoder;
    private final RelativeEncoder m_turningMotorEncoder;
    private final double m_moduleOffset;

    private final PIDController m_drivePIDController = new PIDController(Constants.DRIVE_KP, Constants.DRIVE_KI,
        Constants.DRIVE_KD);
    private final PIDController m_turningPIDController = new PIDController(Constants.STEER_KP, Constants.STEER_KI,
        Constants.STEER_KD);
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.12320, 2.13383); // V,
                                                                                                            // V/(m/s);
                                                                                                            // adjust
                                                                                                            // using
                                                                                                            // SysId

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel     The CAN output for the drive motor.
     * @param turningMotorChannel   The CAN output for the turning motor.
     * @param turningEncoderChannel The CAN input for the turning encoder.
     * @param moduleOffset          The angle offset for the turning encoder (rad).
     */
    private SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel,
        double moduleOffset) {
      m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

      m_driveMotor.restoreFactoryDefaults();
      m_turningMotor.restoreFactoryDefaults();

      m_driveMotor.setIdleMode(IdleMode.kBrake);
      m_turningMotor.setIdleMode(IdleMode.kBrake);

      m_driveMotor.setSmartCurrentLimit(30);
      m_turningMotor.setSmartCurrentLimit(30);

      m_turningMotor.setInverted(true);

      m_driveEncoder = m_driveMotor.getEncoder();
      m_turningCANcoder = new CANcoder(turningEncoderChannel);
      m_turningMotorEncoder = m_turningMotor.getEncoder();

      m_driveEncoder.setPositionConversionFactor(kDriveGearRatio * kWheelRadius * 2 * Math.PI); // Meters
      m_driveEncoder.setVelocityConversionFactor(kDriveGearRatio * kWheelRadius * 2 * Math.PI / 60.0); // Meters per
                                                                                                       // second
      m_turningMotorEncoder.setPositionConversionFactor(kSteerGearRatio * 2 * Math.PI); // Radians
      m_turningMotorEncoder.setVelocityConversionFactor(kSteerGearRatio * 2 * Math.PI / 60.0); // Radians per second

      m_moduleOffset = moduleOffset;

      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

      alignTurningEncoders();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(m_driveEncoder.getVelocity(),
          Rotation2d.fromRotations(m_turningCANcoder.getAbsolutePosition().getValueAsDouble() - m_moduleOffset));
    }

    /**
     * Returns the current distance of the drive encoder in meters as a
     * SwerveModulePosition.
     *
     * @return The current distance of the drive encoder in meters as a
     *         SwerveModulePosition.
     */
    public SwerveModulePosition getDrivePosition() {
      return new SwerveModulePosition(m_driveEncoder.getPosition(),
          Rotation2d.fromRotations(m_turningCANcoder.getAbsolutePosition().getValueAsDouble() - m_moduleOffset));
    }

    /**
     * Returns the drive motor of the module.
     *
     * @return The drive motor of the module.
     */
    public CANSparkMax getDriveMotor() {
      return this.m_driveMotor;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState The desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      // Optimizes the reference state to avoid spinning further than 90 degrees.
      SwerveModuleState state = SwerveModuleState.optimize(desiredState,
          new Rotation2d(m_turningMotorEncoder.getPosition()));
      // Calculates the turning motor output from the variable turning PID controller.
      final double turnOutput = m_turningPIDController.calculate(m_turningMotorEncoder.getPosition(),
          state.angle.getRadians());
      m_turningMotor.set(turnOutput);

      // Updates velocity based on turn error.
      state.speedMetersPerSecond *= Math.cos(getState().angle.getRadians() - state.angle.getRadians());

      // Calculates the drive output from the drive PID controller and feedforward
      // controller.
      final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
          state.speedMetersPerSecond);
      final double driveFeedForward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
      m_driveMotor.setVoltage(Math.abs(state.speedMetersPerSecond) > 0.001 ? (driveFeedForward + driveOutput) : 0);
    }

    /**
     * Sets the relative turning encoder used in PID to the absolute turning encoder
     * position. Do not call periodically.
     */
    public void alignTurningEncoders() {
      m_turningMotorEncoder.setPosition(Rotation2d
          .fromRotations(m_turningCANcoder.getAbsolutePosition().getValueAsDouble() - m_moduleOffset).getRadians());
    }
  }
}