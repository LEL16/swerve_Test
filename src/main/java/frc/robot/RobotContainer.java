package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AutonIntakeCommand;
import frc.robot.Commands.LimelightOuttakeCommand;
import frc.robot.Commands.LimelightRotateCommand;
import frc.robot.Commands.AutonOuttakeCommand;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultClimberCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.DefaultIntakeCommand;
import frc.robot.Commands.DefaultOuttakeCommand;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  private final UsbCamera m_camera;

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private final Joystick m_testController = new Joystick(2);

  private final Joystick m_buttonBoard = new Joystick(2);
  private double m_powerLimit = 0.7;

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
            * DrivetrainSubsystem.kMaxAngularSpeed * 0.5));

    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
        m_intakeSubsystem,
        () -> getDPadInput(m_operatorController) * IntakeSubsystem.kIntakeMaxRate * 0.2,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.05) * IntakeSubsystem.kRotateMaxAngularSpeed
            * 0.75));

    m_outtakeSubsystem.setDefaultCommand(new DefaultOuttakeCommand(
        m_outtakeSubsystem,
        () -> MathUtil.applyDeadband(m_operatorController.getRawAxis(3), 0.05) * OuttakeSubsystem.kOuttakeMaxRate * 0.8,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05)));

    m_climberSubsystem.setDefaultCommand(new DefaultClimberCommand(
        m_climberSubsystem,
        () -> getDPadInput(m_buttonBoard) * MathUtil.applyDeadband(m_buttonBoard.getRawAxis(3), 0.05) * 0.50,
        () -> getDPadInput(m_buttonBoard) * MathUtil.applyDeadband(m_buttonBoard.getRawAxis(2), 0.05) * 0.50));

    m_camera = CameraServer.startAutomaticCapture();
    m_camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    m_camera.setResolution(320, 240);
    Shuffleboard.getTab("Match").add(CameraServer.getServer().getSource()).withWidget(BuiltInWidgets.kCameraStream)
        .withSize(7, 5).withPosition(4, 0);

    configureButtons();
  }

  /**
   * Command sequence to run in autonomous. The origin is the center of the front
   * subwoofer edge.
   * 
   * @param startX          Starting X Position (m).
   * @param startY          Starting Y Position (m).
   * @param startTheta      Starting angle (rad).
   * @param autonomousNotes ArrayList containing notes to be scored in order.
   * @return Command to run autonomously.
   */
  public Command autonomousCommands(double startX, double startY, double startTheta,
      ArrayList<SpikeMarkNote> autonomousNotes) {
    setPose(startX, startY, startTheta);
    m_drivetrainSubsystem.alignTurningEncoders();
    m_intakeSubsystem.reset();

    SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new AutonOuttakeCommand(m_outtakeSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.8, -1.1, 1000),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new AutonIntakeCommand(m_intakeSubsystem, 1000, 0, 500))),
        new AutonOuttakeCommand(m_outtakeSubsystem, 0, -1.5, 500));
    for (SpikeMarkNote note : autonomousNotes) {
      switch (note) {
        case LEFT:
          autonomousSequence.addCommands(leftNoteSequence());
          break;
        case MIDDLE:
          autonomousSequence.addCommands(middleNoteSequence());
          break;
        case RIGHT:
          autonomousSequence.addCommands(rightNoteSequence());
          break;
        case REDAMP:
          autonomousSequence.addCommands(RedMidFieldAmpAuton());
          break;
        case BLUEAMP:
          autonomousSequence.addCommands(BlueMidfieldAmpAuton());
          break;
        case REDSPEAKER:
          autonomousSequence.addCommands(RedMidfieldSpeakerAuton());
          break;
        case BLUESPEAKER:
          autonomousSequence.addCommands(BlueMidfieldSpeakerAuton());
          break;
      }
    }
    return autonomousSequence;
  }

  /**
   * Configures all controller buttons.
   */
  private void configureButtons() {
    // Driver button A
    Trigger m_resetPose = new Trigger(() -> m_driveController.getRawButton(1));
    m_resetPose.onTrue(new InstantCommand(() -> setPose(0, 0, 0)));

    // Operator button A & Button board column 3, row 1
    Trigger m_resetSubsystems = new Trigger(
        () -> (m_operatorController.getRawButton(1) || m_buttonBoard.getRawButton(6)));
    m_resetSubsystems.onTrue(new InstantCommand(() -> m_intakeSubsystem.reset()));

    // Driver button X
    Trigger m_brake = new Trigger(() -> m_driveController.getRawButton(3));
    m_brake.onTrue(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.onFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver D-pad up
    Trigger m_incrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == 1.0);
    m_incrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(0.1)));

    // Driver D-pad down
    Trigger m_decrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == -1.0);
    m_decrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(-0.1)));

    // Button board column 1, row 2
    Trigger m_intake = new Trigger(() -> m_buttonBoard.getRawButton(1));
    m_intake.whileTrue(new AutonIntakeCommand(m_intakeSubsystem, -400, -3.30, 150000));
    m_intake.onFalse(new AutonIntakeCommand(m_intakeSubsystem, 0, .5, 1000));

    // Button board column 1, row 1
    Trigger m_outtakeSpeaker = new Trigger(() -> m_buttonBoard.getRawButton(3));
    m_outtakeSpeaker.onTrue(new ParallelCommandGroup(
        new AutonOuttakeCommand(m_outtakeSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.53, -1.1, 1000),
        new SequentialCommandGroup(
            new WaitCommand(0.5),
            new AutonIntakeCommand(m_intakeSubsystem, 700, 0, 500))));

    // Button board column 2, row 2
    Trigger m_outtakeLimelightSpeaker = new Trigger(() -> m_buttonBoard.getRawButton(2));
    m_outtakeLimelightSpeaker.onTrue(new ParallelCommandGroup(
        new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
            OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1250),
        new SequentialCommandGroup(
            new WaitCommand(0.75),
            new AutonIntakeCommand(m_intakeSubsystem, 700, 0, 500)),
        new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1250)));

    // Button board column 4, row 1
    Trigger m_outtakeAmp = new Trigger(() -> m_buttonBoard.getRawButton(5));
    m_outtakeAmp.onTrue(new SequentialCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, -200, 0, 250),
        new WaitCommand(.25),
        new AutonIntakeCommand(m_intakeSubsystem, 0, -1.32, 400),
        new AutonIntakeCommand(m_intakeSubsystem, IntakeSubsystem.kIntakeMaxRate * 0.66, -1.32, 500),
        new AutonIntakeCommand(m_intakeSubsystem, 0, .5, 275)));

    // Button board column 2, row 1
    Trigger m_cancelSubsystemCommands = new Trigger(() -> m_buttonBoard.getRawButton(4));
    m_cancelSubsystemCommands.onTrue(new InstantCommand(() -> cancelSubsystemCommands()));

    /* Temporary Testing Triggers */
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
  }

  /**
   * Cancels all subsystem commands and presets.
   */
  private void cancelSubsystemCommands() {
    if (m_intakeSubsystem.getCurrentCommand() != null) {
      m_intakeSubsystem.getCurrentCommand().cancel();
    }
    if (m_outtakeSubsystem.getCurrentCommand() != null) {
      m_outtakeSubsystem.getCurrentCommand().cancel();
    }
  }

  /**
   * Sets robot odometric position.
   * 
   * @param xPos  X Position (m).
   * @param yPos  Y Position (m).
   * @param theta Angle (rad).
   */
  private void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
    m_drivetrainSubsystem.alignTurningEncoders();
  }

  /**
   * Changes the drive controller power limit [0, 1].
   * 
   * @param delta Power Limit Change.
   */
  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }

  /**
   * Converts D-Pad input into either +1 or -1 output.
   * 
   * @param joystick Joystick to access D-Pad.
   * @return D-Pad up returns +1 and D-Pad down returns -1.
   */
  private double getDPadInput(Joystick joystick) {
    if (joystick.getPOV() >= 315 || (joystick.getPOV() <= 45 && joystick.getPOV() >= 0)) {
      return 1.0;
    }
    if (joystick.getPOV() >= 135 && joystick.getPOV() <= 225) {
      return -1.0;
    }
    return 0;
  }

  private Command leftNoteSequence() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new AutonOuttakeCommand(m_outtakeSubsystem, 0, -1.4, 1000),
            new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 1.50, 0, 3, Math.PI / 2, 1000),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new AutonIntakeCommand(m_intakeSubsystem, 0, -2.8, 500))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -500, -3.3, 1000),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                new PositionDriveCommand(m_drivetrainSubsystem, 1.85, 1.50, 0, 750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 750),
            new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 1.00, 0.460, 750)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1250),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new AutonIntakeCommand(m_intakeSubsystem, 700, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1250)));
  }

  /**
   * Command to intake and shoot the note on the middle spike mark of either
   * alliance, from the POV of the drivers.
   * 
   * @return Command to intake and shoot the note on the middle spike mark of
   *         either alliance, from the POV of the drivers.
   */
  private Command middleNoteSequence() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 0, 0, 3, Math.PI / 2, 1000),
            new AutonIntakeCommand(m_intakeSubsystem, 0, -2.8, 1000)),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -500, -3.3, 1000),
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new PositionDriveCommand(m_drivetrainSubsystem, 1.80, 0, 0, 750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 750),
            new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 0, 0, 750)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1000),
            new SequentialCommandGroup(
                new WaitCommand(.75),
                new AutonIntakeCommand(m_intakeSubsystem, 700, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)));
  }

  /**
   * Command to intake and shoot the note on the right-most spike mark of either
   * alliance, from the POV of the drivers.
   * 
   * @return Command to intake and shoot the note on the right-most spike mark of
   *         either alliance, from the POV of the drivers.
   */
  private Command rightNoteSequence() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new AutonOuttakeCommand(m_outtakeSubsystem, 0, -1.4, 1000),
            new PositionDriveCommand(m_drivetrainSubsystem, 1.10, -1.50, 0, 3, Math.PI / 2, 1000),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new AutonIntakeCommand(m_intakeSubsystem, 0, -2.8, 500))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -500, -3.3, 1000),
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new PositionDriveCommand(m_drivetrainSubsystem, 1.80, -1.50, 0, 750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 750),
            new PositionDriveCommand(m_drivetrainSubsystem, 1.10, -1.00, -0.460, 750)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1250),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1250)));
  }

  /**
   * Command to intake and shoot the note on the Blue Ampside Midfield Note, the
   * one closest to the wall and the one adjacent.
   * NON-TESTED
   * 
   * @return Command to intake and shoot the note on the right-most spike mark of
   *         either alliance, from the POV of the drivers.
   */
  private Command BlueMidfieldAmpAuton() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 1.4064, 1.8156, 0, 4, Math.PI / 2, 800),
            new AutonIntakeCommand(m_intakeSubsystem, 0, -2.5, 800)),
        new PositionDriveCommand(m_drivetrainSubsystem, 2.4064, 1.8156, 0, 4, Math.PI / 2, 300),

        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134 + .5, .1254 - .209, 0, 4, Math.PI / 2, 1800),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 2000),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 2000),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.7022, .7733, .23, 4, Math.PI / 2, 2000)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)),
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134 + .5, 1.5675, 0, 4, Math.PI / 2, 2000),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 1500),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 1500),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.7022, .7733, .23, 4, Math.PI / 2, 1750)),

        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)));
  }

  /**
   * Command to intake and shoot the note on the Red Ampside Midfield Note, the
   * one closest to the wall and the one adjacent.
   * NON-TESTED
   * 
   * @return Command to intake and shoot the note on the right-most spike mark of
   *         either alliance, from the POV of the drivers.
   */
  private Command RedMidFieldAmpAuton() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 1.4064, -1.8156, 0, 4, Math.PI / 2, 1000),
            new AutonIntakeCommand(m_intakeSubsystem, 0, -2.5, 1000)),
        new PositionDriveCommand(m_drivetrainSubsystem, 2.4064, -1.8156, 0, 4, Math.PI / 2, 300),

        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134 + .5, -.1254 - .209, 0, 4, Math.PI / 2, 2000),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 2000),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 2000),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.7022, -.7733, -.23, 4, Math.PI / 2, 2000)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)),
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134 + .5, -1.5675, 0, 4, Math.PI / 2, 2000),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1500))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 1500),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 1500),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.7022, -.7733, -.23, 4, Math.PI / 2, 1750)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)));
  }

  /**
   * Command to intake and shoot the note on the Blue Speakerside Midfield Note,
   * the one closest to the wall and the one adjacent.
   * NON-FUNCTIONAL
   * 
   * @return Command to intake and shoot the note on the right-most spike mark of
   *         either alliance, from the POV of the drivers.
   */
  private Command BlueMidfieldSpeakerAuton() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 1.4064, -4.0514, 0, 4, Math.PI / 2, 2500),
            new AutonIntakeCommand(m_intakeSubsystem, 0, -2.5, 1500)),

        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134, -2.6125, 0, 4, Math.PI / 2, 2000),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 2000),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 2000),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.3678, -2.0064, -.23, 4, Math.PI / 2, 2500)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)),
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134, -3.9919, 0, 4, Math.PI / 2, 2500),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 2000),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 2000),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.3678, -2.0064, -.23, 4, Math.PI / 2, 2500)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)));
  }

  /**
   * Command to intake and shoot the note on the Red Speakerside Midfield Note,
   * the one closest to the wall and the one adjacent.
   * NON-FUNCTIONAL
   * 
   * @return Command to intake and shoot the note on the right-most spike mark of
   *         either alliance, from the POV of the drivers.
   */
  private Command RedMidfieldSpeakerAuton() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 1.4064, 4.0514, 0, 4, Math.PI / 2, 2500),
            new AutonIntakeCommand(m_intakeSubsystem, 0, -2.5, 1500)),

        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134, 2.6125, 0, 4, Math.PI / 2, 2000),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 2000),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 2000),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.3678, 2.0064, .23, 4, Math.PI / 2, 2500)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)),
        new ParallelCommandGroup(
            new PositionDriveCommand(m_drivetrainSubsystem, 6.2134, 3.9919, 0, 4, Math.PI / 2, 2500),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new AutonIntakeCommand(m_intakeSubsystem, -800, -3.3, 1750))),
        new ParallelCommandGroup(
            new AutonIntakeCommand(m_intakeSubsystem, -100, 1, 2000),
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem, 0, 2000),
            new PositionDriveCommand(m_drivetrainSubsystem, 2.3678, 2.0064, .23, 4, Math.PI / 2, 2500)),
        new ParallelCommandGroup(
            new LimelightOuttakeCommand(m_drivetrainSubsystem, m_outtakeSubsystem, m_limelightSubsystem,
                OuttakeSubsystem.kOuttakeMaxRate * 0.8, 1500),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new AutonIntakeCommand(m_intakeSubsystem, 800, 0, 500)),
            new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1000)));
  }

  /* Autonomous spike mark note options */
  enum SpikeMarkNote {
    LEFT,
    MIDDLE,
    RIGHT,
    REDAMP,
    BLUEAMP,
    REDSPEAKER,
    BLUESPEAKER
  }
}