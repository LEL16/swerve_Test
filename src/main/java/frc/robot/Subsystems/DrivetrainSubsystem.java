// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Utils.AllianceUtil;
import frc.robot.Utils.LimelightHelpers;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
  private static final double kTrackWidth = 0.60; // meters

  public static final double kMaxSpeed = (5676.0 / 60.0) * SwerveModule.kDriveGearRatio * SwerveModule.kWheelRadius * 2
      * Math.PI; // meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kTrackWidth / 2.0); // radians
                                                                                                              // per
                                                                                                              // second

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

  private final SwerveDrivePoseEstimator m_poseEstimator;

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
  private final GenericEntry m_distanceToNearestSpeakerEntry;
  private final GenericEntry m_outtakeAngleEntry;
  private final GenericEntry m_drivetrainAngleChangeEntry;
  private final Field2d m_field;

  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rot;
  private boolean m_fieldRelative;

  private double m_poseDistance;
  private double m_poseAngle;

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

    // m_odometry = new SwerveDriveOdometry(m_kinematics, m_navx.getRotation2d(),
    // getModulePositions());

    m_field = new Field2d();

    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics,
        m_navx.getRotation2d(),
        getModulePositions(),
        new Pose2d(0.0, 0.0, m_navx.getRotation2d()));

    AutoBuilder.configureHolonomic(
        () -> new Pose2d(getPosition(), getAngle()),
        (pose) -> setPose(pose.getX(), pose.getY(), pose.getRotation().getRadians()),
        () -> m_kinematics.toChassisSpeeds(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState(),
        }),
        (chassisSpeed) -> drive(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond,
            chassisSpeed.omegaRadiansPerSecond, false),
        new HolonomicPathFollowerConfig(
            new PIDConstants(2.0, 0.1, 0.1), // Trans
            new PIDConstants(2.5, 0.2, 0.1), // Rot
            3.81,
            kTrackWidth,
            new ReplanningConfig(false, true)),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
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

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    ShuffleboardLayout frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(0, 0);
    m_frontLeftDriveSpeedEntry = frontLeftLayout
        .add("Drive Speed", m_frontLeft.getState().speedMetersPerSecond + " m/s").getEntry();
    m_frontLeftSteerAngleEntry = frontLeftLayout.add("Steer Angle", m_frontLeft.getState().angle.getDegrees() + " deg")
        .getEntry();

    ShuffleboardLayout frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(2, 0);
    m_frontRightDriveSpeedEntry = frontRightLayout
        .add("Drive Speed", m_frontRight.getState().speedMetersPerSecond + " m/s").getEntry();
    m_frontRightSteerAngleEntry = frontRightLayout
        .add("Steer Angle", m_frontRight.getState().angle.getDegrees() + " deg").getEntry();

    ShuffleboardLayout backLeftLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(4, 0);
    m_backLeftDriveSpeedEntry = backLeftLayout.add("Drive Speed", m_backLeft.getState().speedMetersPerSecond + " m/s")
        .getEntry();
    m_backLeftSteerAngleEntry = backLeftLayout.add("Steer Angle", m_backLeft.getState().angle.getDegrees() + " deg")
        .getEntry();

    ShuffleboardLayout backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 2)
        .withPosition(6, 0);
    m_backRightDriveSpeedEntry = backRightLayout
        .add("Drive Speed", m_backRight.getState().speedMetersPerSecond + " m/s").getEntry();
    m_backRightSteerAngleEntry = backRightLayout.add("Steer Angle", m_backRight.getState().angle.getDegrees() + " deg")
        .getEntry();

    ShuffleboardLayout odometryLayout = tab.getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 3).withPosition(0,
        2);
    m_odometryXEntry = odometryLayout.add("X Position", getPosition().getX() + " m").getEntry();
    m_odometryYEntry = odometryLayout.add("Y Position", getPosition().getY() + " m").getEntry();
    m_odometryThetaEntry = odometryLayout.add("Angle", getAngle().getDegrees() + " deg").getEntry();

    ShuffleboardLayout limelightLayout = tab.getLayout("Limelight", BuiltInLayouts.kList).withSize(2, 3).withPosition(6,
        0);
    m_distanceToNearestSpeakerEntry = limelightLayout.add("Distance to Nearest Speaker", getDistanceToSpeaker() + " m")
        .getEntry();
    m_outtakeAngleEntry = limelightLayout.add("Desired Outtake Angle", calculateOuttakeAngle() + " rad").getEntry();
    m_drivetrainAngleChangeEntry = limelightLayout.add("Desired Drivetrain Angle Change", getAngleToSpeaker() + " rad")
        .getEntry();

    tab.add(m_field);
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
    return m_poseEstimator.getEstimatedPosition().getTranslation();
  }

  /**
   * Returns the current odometric angle of the robot.
   * 
   * @return The current odometric angle of the robot.
   */
  public Rotation2d getAngle() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Sets the odometric position and angle of the robot.
   *
   * @param xPos  The position of the robot in the x direction (m).
   * @param yPos  The position of the robot in the y direction (m).
   * @param theta The angle of the robot (rad).
   */
  public void setPose(double xPos, double yPos, double theta) {
    m_poseEstimator.resetPosition(m_navx.getRotation2d(), getModulePositions(),
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
   * Returns a command that will execute a quasistatic test in the given
   * direction.
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

  /**
   * Calculates the distance (m) to the speaker robot-relative.
   * 
   * @return Robot distance to speaker (m).
   */
  public double getDistanceToSpeaker() {
    return m_poseDistance;
  }

  /**
   * Calculates the angle (rad) to the speaker robot-relative.
   * 
   * @return Robot angle to speaker (rad).
   */
  public double getAngleToSpeaker() {
    return m_poseAngle;
  }

  /**
   * Calculates the optimal outtake angle for shooting based on limelight
   * trigonometric input.
   * 
   * @return Optimal outtake absolute angle (rad).
   */
  public double calculateOuttakeAngle() {
    if (getDistanceToSpeaker() > 3.4) {
      // Slightly modified regression for long distances (REALLY GOOD)!
      return 1.12053 * Math.pow(getDistanceToSpeaker(), -0.877924) - 1.8339;
    }
    return 1.12053 * Math.pow(getDistanceToSpeaker(), -0.877924) - 1.8539;
    // Original line: 1.12053 * Math.pow(getDistanceToNearestSpeaker(),-.877924) -
    // 1.8639
  }

  /** Updates the odometry of the robot. */
  public void updateOdometry() {
    if (!LimelightHelpers.getTV("limelight")) {
      return;
    }

    m_poseEstimator.update(m_navx.getRotation2d(), getModulePositions());

    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (limelightMeasurement.tagCount == 0) {
      return;
    }

    if (limelightMeasurement.tagCount == 1) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7,9999999)); // Increase to trust vision less
    } else if (limelightMeasurement.tagCount >= 2) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999)); // Increase to trust vision less
    } else {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1.0, 1.0, 9999999)); // Increase to trust vision less
    }

    m_poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
  }

  /** Displays the periodically updated robot poses on the Shuffleboard */
  public void updateShuffleboard() {
    m_frontLeftDriveSpeedEntry.setString(m_frontLeft.getState().speedMetersPerSecond + " m/s");
    m_frontLeftSteerAngleEntry.setString(m_frontLeft.getState().angle.getDegrees() + " deg");

    m_frontRightDriveSpeedEntry.setString(m_frontRight.getState().speedMetersPerSecond + " m/s");
    m_frontRightSteerAngleEntry.setString(m_frontRight.getState().angle.getDegrees() + " deg");

    m_backLeftDriveSpeedEntry.setString(m_backLeft.getState().speedMetersPerSecond + " m/s");
    m_backLeftSteerAngleEntry.setString(m_backLeft.getState().angle.getDegrees() + " deg");

    m_backRightDriveSpeedEntry.setString(m_backRight.getState().speedMetersPerSecond + " m/s");
    m_backRightSteerAngleEntry.setString(m_backRight.getState().angle.getDegrees() + " deg");

    m_odometryXEntry.setString(getPosition().getX() + " m");
    m_odometryYEntry.setString(getPosition().getY() + " m");
    m_odometryThetaEntry.setString(getAngle().getDegrees() + " deg");

    m_distanceToNearestSpeakerEntry.setString(getDistanceToSpeaker() + " m");
    m_outtakeAngleEntry.setString(calculateOuttakeAngle() + " rad");
    m_drivetrainAngleChangeEntry.setString(getAngleToSpeaker() + "rad");

    m_field.setRobotPose(new Pose2d(this.getPosition(), this.getAngle()));
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

    m_poseDistance = Math.hypot(
        Math.abs(this.getPosition().getX() - AllianceUtil
            .getSpeakerPose("blue", new Pose2d(this.getPosition(), this.getAngle())).getTranslation().getX()),
        Math.abs(this.getPosition().getY() - AllianceUtil
            .getSpeakerPose("blue", new Pose2d(this.getPosition(), this.getAngle())).getTranslation().getY()));

    double m_distanceX = this.getPosition().getX()
        - AllianceUtil.getSpeakerPose("blue", new Pose2d(this.getPosition(), this.getAngle())).getTranslation().getX();
    double m_distanceY = this.getPosition().getY()
        - AllianceUtil.getSpeakerPose("blue", new Pose2d(this.getPosition(), this.getAngle())).getTranslation().getY();
    m_poseAngle = Math.atan(m_distanceY / m_distanceX);

    updateOdometry();
    updateShuffleboard();
  }

  private class SwerveModule {
    // private static final double kWheelRadius = Units.inchesToMeters(2.0);
    // private static final double kDriveGearRatio = 1 / 6.12;
    private static final double kWheelRadius = 0.050165; // meters
    private static final double kDriveGearRatio = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    private static final double kSteerGearRatio = 7.0 / 150.0;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final CANcoder m_turningCANcoder;
    private final RelativeEncoder m_turningMotorEncoder;
    private final double m_moduleOffset;

    private final PIDController m_drivePIDController = new PIDController(0, 0, 0);
    private final PIDController m_turningPIDController = new PIDController(0.5, 0, 0.01);
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 2.4);

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

      m_driveMotor.setIdleMode(IdleMode.kBrake);
      m_turningMotor.setIdleMode(IdleMode.kBrake);

      m_turningMotor.setInverted(true);

      m_driveEncoder = m_driveMotor.getEncoder();
      m_turningCANcoder = new CANcoder(turningEncoderChannel);
      m_turningMotorEncoder = m_turningMotor.getEncoder();

      m_driveEncoder.setPositionConversionFactor(kDriveGearRatio * kWheelRadius * 2 * Math.PI); // meters
      m_driveEncoder.setVelocityConversionFactor(kDriveGearRatio * kWheelRadius * 2 * Math.PI / 60.0); // meters per
                                                                                                       // second

      m_turningMotorEncoder.setPositionConversionFactor(kSteerGearRatio * 2 * Math.PI); // radians
      m_turningMotorEncoder.setVelocityConversionFactor(kSteerGearRatio * 2 * Math.PI / 60.0); // radians per second

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
     * Returns the drive motor of the module.
     *
     * @return The drive motor of the module.
     */
    public CANSparkMax getDriveMotor() {
      return this.m_driveMotor;
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