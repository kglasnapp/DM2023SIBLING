// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Util.logf;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultGrabberCommand;
import frc.robot.commands.PositionCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberTiltSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PoseEstimatorAggregator;
import frc.robot.utilities.SwerveModule;
//import frc.robot.Autonomous;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // robotIDCheck.get returns true for the sibling, false for
  final DigitalInput robotIDCheck = new DigitalInput(0);

  public final GrabberTiltSubsystem grabberSubsystem = new GrabberTiltSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(grabberSubsystem);
  public final static LedSubsystem leds = new LedSubsystem();
  private final static CommandXboxController driveController = new CommandXboxController(2);
  private final static CommandXboxController operatorController = new CommandXboxController(3);
  private final Autonomous autotonomous = new Autonomous(m_drivetrainSubsystem);
  public boolean USBCamera = false;
  private final BalanceCommand balanceCommand = new BalanceCommand(m_drivetrainSubsystem);
  private SlewRateLimiter sLX = new SlewRateLimiter(9);
  private SlewRateLimiter sLY = new SlewRateLimiter(9);
  private SlewRateLimiter sRX = new SlewRateLimiter(1);
  public static SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  public static  boolean smartForElevator = true;
  public static RobotMode robotMode;
  public static boolean smartDashBoardForElevator = true;

  public static ShowPID showPID = ShowPID.TILT;

  public static enum RobotMode {
    Cone, Cube
  }

  public static enum ShowPID {
    NONE, ELEVATOR, TILT
  }

  public static enum OperatorButtons {
    HOME(3), CUBE(2), CONE(1), GROUND(4), CHUTE(5), SHELF(6), LOW(7), MIDDLE(8), HIGH(9);

    public final int value;

    private OperatorButtons(int value) {
      this.value = value;
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set the default Robot Mode to Cube
    setMode(RobotMode.Cube);
    grabberSubsystem.setDefaultCommand(new DefaultGrabberCommand(grabberSubsystem, intakeSubsystem, driveController));
    Command defaulElevatorCommand = new DefaultElevatorCommand(elevatorSubsystem, grabberSubsystem, driveController);
    elevatorSubsystem.setDefaultCommand(defaulElevatorCommand);

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> (SwerveModule.powerRatio == 1 ? -modifyAxis((sLY.calculate(driveController.getLeftY())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((driveController.getLeftY()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModule.powerRatio == 1 ? -modifyAxis((sLX.calculate(driveController.getLeftX())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((driveController.getLeftX()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModule.powerRatio == 1
            ? -modifyAxis((sRX.calculate(driveController.getRightX())))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((driveController.getRightX()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        // Set precision based upon left bumper
        driveController.leftBumper()));
    configureButtonBindings();
    configureDashboard();
  }

  public double squareWithSign(double v) {
    return (v < 0) ? -(v * v) : (v * v);
  }

  private void configureDashboard() {
  }

  public static double getRightTrigger() {
    return driveController.getRightTriggerAxis();
  }

  public static double getLeftTrigger() {
    return driveController.getLeftTriggerAxis();
  }

  public static boolean getRightBumper() {
    return driveController.getHID().getRawButton(6);
  }

  public static boolean getLeftBumper() {
    return driveController.getHID().getRawButton(5);
  }

  public static int getDriverPov() {
    return driveController.getHID().getPOV();
  }

  public void setMode(RobotMode mode) {
    robotMode = mode;
    logf("Set Robot Mode: %s\n", mode);
    SmartDashboard.putString("Mode", robotMode.toString());
    leds.setRobotModeLeds();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new Button(m_controller::getAButton).whileHeld(holonomicTargetCommand);
    // new Button(m_controller::getBButton).whileHeld(chaseTagCommand);
    // Back button zeros the gyroscope);
    driveController.back().onTrue(new RunCommand(new Runnable() {
      public void run() {
        m_drivetrainSubsystem.zeroGyroscope();
      }
    }));

    driveController.start().onTrue(new RunCommand(new Runnable() {
      public void run() {
        balanceCommand.zeroGyroscope();
      }
    }));

    operatorController.button(OperatorButtons.CONE.value).onTrue(Commands.runOnce(new Runnable() {
      //   controller.button(OperatorButtons.CONE.value).onTrue((new Runnable() {
      public void run() {
        setMode(RobotMode.Cone);
      }
    }));

    operatorController.button(OperatorButtons.CUBE.value).onTrue(Commands.runOnce(new Runnable() {
      //   controller.button(OperatorButtons.CUBE.value).onTrue(new RunCommand(new Runnable() {
      public void run() {
        setMode(RobotMode.Cube);
      }
    }));

    driveController.start().onTrue(Commands.runOnce(new Runnable() {
      public void run() {
        if (showPID == ShowPID.TILT) {
          showPID = ShowPID.ELEVATOR;
        } else {
          showPID = ShowPID.TILT;
        }
        logf("Change PID control to %s\n", showPID);
      }
    }));

    operatorController.button(OperatorButtons.LOW.value).onTrue(new PositionCommand(this, OperatorButtons.LOW));
    operatorController.button(OperatorButtons.MIDDLE.value).onTrue(new PositionCommand(this, OperatorButtons.MIDDLE));
    operatorController.button(OperatorButtons.HIGH.value).onTrue(new PositionCommand(this, OperatorButtons.HIGH));
    operatorController.button(OperatorButtons.HOME.value).onTrue(new PositionCommand(this, OperatorButtons.HOME));

    operatorController.button(OperatorButtons.GROUND.value).onTrue(new PositionCommand(this, OperatorButtons.GROUND));
    operatorController.button(OperatorButtons.CHUTE.value).onTrue(new PositionCommand(this, OperatorButtons.CHUTE));
    operatorController.button(OperatorButtons.SHELF.value).onTrue(new PositionCommand(this, OperatorButtons.SHELF));

    // y Button will rotate the Robot 180 degrees
    driveController.y().onTrue(new RotateCommand(m_drivetrainSubsystem));
  }

  public Pose2d getPickupPose(boolean right, int phase) {
    var alliance = DriverStation.getAlliance();
    Pose2d pose = null;
    if (alliance == Alliance.Blue) {
      if (right) {
        pose = new Pose2d(new Translation2d(15.14, 5.95), new Rotation2d(Math.toRadians(0)));
      } else {
        pose = new Pose2d(new Translation2d(15.14, 7.50), new Rotation2d(Math.toRadians(0)));
      }
      if (phase == 1) {
        pose = new Pose2d(pose.getX() - 0.36, pose.getY(), pose.getRotation());
      }
    } else {
      if (right) {
        pose = new Pose2d(new Translation2d(15.14, 0.55), new Rotation2d(Math.toRadians(0)));
      } else {
        pose = new Pose2d(new Translation2d(15.14, 1.88), new Rotation2d(Math.toRadians(0)));
      }
      if (phase == 1) {
        pose = new Pose2d(pose.getX() - 0.36, pose.getY(), pose.getRotation());
      }
    }
    return pose;
  }

  Supplier<Pose2d> getPoseEstimatorForTarget(PoseEstimatorAggregator poseEstimatorAggregator, int pos) {
    return poseEstimatorAggregator;
  }

  Pose2d getFinalPoseForCase1(int state) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      if (state == 0) {
        return new Pose2d(6.4,
            4.5,
            new Rotation2d(Math.toRadians(180)));
      } else {
        return new Pose2d(6.4,
            0.92,
            new Rotation2d(Math.toRadians(180)));
      }
    } else {
      if (state == 0) {
        return new Pose2d(6.4,
            1.096, // just changed this value at 4:54
            new Rotation2d(Math.toRadians(180)));
      } else {
        return new Pose2d(6.4,
            3.42,
            new Rotation2d(Math.toRadians(180)));
      }
    }
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.08);
    value = Math.copySign(value * value, value); // Square the axis
    return value;
  }
}
