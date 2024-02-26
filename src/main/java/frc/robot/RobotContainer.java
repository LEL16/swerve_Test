package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.Intake.AutonIntakeCommand;
import frc.robot.Commands.Outtake.AutonOuttakeCommand;
import frc.robot.Commands.Drive.BrakeCommand;
// import frc.robot.Commands.Climber.DefaultClimberCommand;
import frc.robot.Commands.Drive.DefaultDriveCommand;
import frc.robot.Commands.Intake.DefaultIntakeCommand;
import frc.robot.Commands.Limelight.LimelightAlignmentCommand;
import frc.robot.Commands.Limelight.PoseAlignmentCommand;
import frc.robot.Commands.Outtake.DefaultOuttakeCommand;
import frc.robot.Commands.Drive.IdleDriveCommand;
import frc.robot.Commands.Drive.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private final Joystick m_testController = new Joystick(2);

  // private final GenericHID m_operatorButtonPad = new GenericHID(2);

  private boolean JoystickOperator = true; // Used for testing ButtonPad.
  private double m_powerLimit = 1.0;

  private static final double kIntakeGearRatio = 1;
  public static final double kIntakeAutonRate = 5676.0 * kIntakeGearRatio * 0.5; // rpm

  private static final double kRotateGearRatio = (1.0 / 20.0);
  public static final double kRotateAutonAngularSpeed = 5676.0 * 0.5 * kRotateGearRatio * 2.0 * Math.PI / 60; // rad/s

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Field2d m_field;

  /**
   * This class stores all robot related subsystems, commands, and methods that
   * the {@link Robot} class can utilize during different OpModes.
   */
  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxAngularSpeed * 0.5
    ));

    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
        m_intakeSubsystem, 
        () -> getDPadInput(m_operatorController) * IntakeSubsystem.kIntakeMaxRate * 0.15, 
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.05) * IntakeSubsystem.kRotateMaxAngularSpeed * 0.1
    ));

    m_outtakeSubsystem.setDefaultCommand(new DefaultOuttakeCommand(
        m_outtakeSubsystem, 
        () -> MathUtil.applyDeadband(m_operatorController.getRawAxis(3), 0.05) * OuttakeSubsystem.kOuttakeMaxRate,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05)
    ));

    // else if (JoystickOperator == false) {
    // m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
    // m_intakeSubsystem,
    // () -> {
    // if (m_operatorButtonPad.getRawButton(5)) {
    // return -1 * IntakeSubsystem.kIntakeMaxRate * 0.5; // counterclockwise on
    // button "lb"
    // }
    // else if (m_operatorButtonPad.getRawButton(6)) {
    // return IntakeSubsystem.kIntakeMaxRate * 0.5; // clockwise on button "rb"
    // }
    // return 0;
    // },
    // () -> {
    // int POVangle = m_operatorButtonPad.getPOV(); // use POV left/right for
    // raising and lowering intake, use up/down for variable shooting
    // if (POVangle == 90){
    // return IntakeSubsystem.kRotateMaxAngularSpeed * 0.2; // right POV
    // }
    // else if (POVangle == 270){
    // return -1 * IntakeSubsystem.kRotateMaxAngularSpeed * 0.2; // left POV
    // }
    // return 0;
    // },
    // () -> m_operatorController.getRawButton(3)
    // ));
    // m_outtakeSubsystem.setDefaultCommand(new DefaultOuttakeCommand(
    // m_outtakeSubsystem,
    // () -> {
    // if (m_operatorController.getRawButton(1)) {
    // return OuttakeSubsystem.kOuttakeMaxRate; // button "a"
    // }
    // else if(m_operatorController.getRawButton(2)){
    // return -1 * OuttakeSubsystem.kOuttakeMaxRate; // button "b"
    // }
    // return 0;
    // }
    // ));
    // }

    m_field = new Field2d();

    ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

    autonomousTab.add("Field", m_field);
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      m_field.setRobotPose(pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_field.getObject("path").setPoses(poses);
    });

    NamedCommands.registerCommand("Intake Note",
        new AutonIntakeCommand(m_intakeSubsystem, kIntakeAutonRate, 2));
    NamedCommands.registerCommand("Outtake Note",
        new AutonOuttakeCommand(m_outtakeSubsystem, kIntakeAutonRate, 2));

    autoChooser = AutoBuilder.buildAutoChooser("DefaultAuton"); // Default path
    autonomousTab.add("Auto Chooser", autoChooser);

    configureButtons();

    CommandScheduler.getInstance().schedule(m_drivetrainSubsystem.getDefaultCommand());
    CommandScheduler.getInstance().schedule(m_intakeSubsystem.getDefaultCommand());
    CommandScheduler.getInstance().schedule(m_outtakeSubsystem.getDefaultCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // Currently used for testing kinematics
  // public Command autonomousCommands() {
  // m_powerLimit = 1.0;
  // // m_intakeSubsystem.reset();
  // return new SequentialCommandGroup(
  // new PositionDriveCommand(m_drivetrainSubsystem, 1.0, 0.5, Math.PI / 2, 2.5,
  // Math.PI, 1500),
  // new PositionDriveCommand(m_drivetrainSubsystem, 2.0, 0, 0, 2.5, Math.PI,
  // 1500),
  // new PositionDriveCommand(m_drivetrainSubsystem, 1.0, -0.5, -Math.PI / 2, 2.5,
  // Math.PI, 1500),
  // new PositionDriveCommand(m_drivetrainSubsystem, 0, 0, 0, 2.5, Math.PI, 1500)
  // );
  // }

  private void configureButtons() {
    // Driver button A
    Trigger m_resetPose = new Trigger(() -> m_driveController.getRawButton(1));
    m_resetPose.onTrue(new InstantCommand(() -> setPose(0, 0, 0)));

    // Driver button X
    Trigger m_brake = new Trigger(() -> m_driveController.getRawButton(3));
    m_brake.onTrue(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.onFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver D-pad up
    Trigger m_incrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == 1.0);

    // Driver D-pad down
    Trigger m_decrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == -1.0);

    // Bind full set of SysId routine tests to buttons
    // A complete routine should run each of these once
    // Test Button A
    Trigger m_quasiForward = new Trigger(() -> m_testController.getRawButton(1));
    m_quasiForward.onTrue(m_drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    // Test Button B
    Trigger m_quasiReverse = new Trigger(() -> m_testController.getRawButton(2));
    m_quasiReverse.onTrue(m_drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // Test Button X
    Trigger m_dynamicForward = new Trigger(() -> m_testController.getRawButton(3));
    m_dynamicForward.onTrue(m_drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

    // Test Button Y
    Trigger m_dynamicReverse = new Trigger(() -> m_testController.getRawButton(4));
    m_dynamicReverse.onTrue(m_drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Driver Button LB
    Trigger m_limelightRotationalAlignment = new Trigger(() -> m_driveController.getRawButton(6));
    m_limelightRotationalAlignment.whileTrue(new LimelightAlignmentCommand(m_drivetrainSubsystem, m_limelightSubsystem,
        "rotational",
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed));

    // Driver Button RB
    Trigger m_poseRotationalAlignment = new Trigger(() -> m_driveController.getRawButton(5));
    m_poseRotationalAlignment.whileTrue(new PoseAlignmentCommand(m_drivetrainSubsystem,
        () -> new Pose2d(m_drivetrainSubsystem.getPosition(), m_drivetrainSubsystem.getAngle()),
        new Pose2d(0, 5.50, new Rotation2d(0))));
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
    m_drivetrainSubsystem.alignTurningEncoders();
  }

  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }

  private double getDPadInput(Joystick joystick) {
    if (joystick.getPOV() >= 315 || (joystick.getPOV() <= 45 && joystick.getPOV() >= 0)) {
      return 1.0;
    }
    if (joystick.getPOV() >= 135 && joystick.getPOV() <= 225) {
      return -1.0;
    }
    return 0;
  }

}
