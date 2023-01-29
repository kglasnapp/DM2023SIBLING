// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.GamePiecePlacementCommand;
import frc.robot.commands.HolonomicTargetCommand;
import frc.robot.commands.PathFollowCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
//import static frc.robot.utilities.Util.logf;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // private final XboxController m_controller = new XboxController(2);
  private final static CommandXboxController m_controller = new CommandXboxController(2);
  
  public PhotonCamera photonCameras[] = { new PhotonCamera("gloworm") };

  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCameras, m_drivetrainSubsystem);

  private final GamePiecePlacementCommand gamePiecePlacementCommand = new GamePiecePlacementCommand(m_drivetrainSubsystem, 
    m_armSubsystem, poseEstimator, GamePiecePlacementCommand.driveTrainPoseTargets[0], GamePiecePlacementCommand.armTargets[0]);
  // private final PathFollowCommand pathFollowCommand = new PathFollowCommand(m_drivetrainSubsystem,
  //     poseEstimator::getCurrentPose);

  private final BalanceCommand balanceCommand = new BalanceCommand(m_drivetrainSubsystem);

  public static boolean mrKeith = false;
  private SlewRateLimiter sLX = new SlewRateLimiter(3);
  private SlewRateLimiter sLY = new SlewRateLimiter(3);
  private SlewRateLimiter sRX = new SlewRateLimiter(3);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //     m_drivetrainSubsystem,
    //     () -> -modifyAxis(squareWithSign(m_controller.getLeftY())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(squareWithSign( m_controller.getLeftX())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(squareWithSign(m_controller.getRightX()))
    //         * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(squareWithSign(sLY.calculate(m_controller.getLeftY()))) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(squareWithSign( sLX.calculate(m_controller.getLeftX()))) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(squareWithSign( sRX.calculate(m_controller.getRightX())))
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            m_controller.x()));
    m_armSubsystem.setDefaultCommand(new DefaultArmCommand(m_armSubsystem,
        () -> (RobotContainer.getLeftBumper() ? -1 : 1) * RobotContainer.getLeftTrigger(),
        () -> (RobotContainer.getRightBumper() ? -1 : 1) * RobotContainer.getRightTrigger()));
    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();
  }

  public double squareWithSign(double v) {
    return (v < 0) ? -(v * v) : (v * v);
  }

  private void configureDashboard() {
  }

  public static double getRightTrigger() {
    return m_controller.getRightTriggerAxis();
  }

  public static double getLeftTrigger() {
    return m_controller.getLeftTriggerAxis();
  }

  public static boolean getRightBumper() {
    return m_controller.getHID().getRawButton(6);
  }

  public static boolean getLeftBumper() {
    return m_controller.getHID().getRawButton(5);
  }

  public static int getPov() {
    return m_controller.getHID().getPOV();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    // new Button(m_controller::getBackButton)
    // .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    // new Button(m_controller::getAButton).whileHeld(holonomicTargetCommand);
    // new Button(m_controller::getBButton).whileHeld(chaseTagCommand);

    m_controller.back().whileTrue(new RunCommand(new Runnable() {
      public void run() {
        m_drivetrainSubsystem.zeroGyroscope();
      }
    }));

    m_controller.start().whileTrue(new RunCommand(new Runnable() {
      public void run() {
        balanceCommand.zeroGyroscope();
      }
    }));

    // m_controller.a().whileTrue(holonomicTargetCommand);
    // m_controller.b().whileTrue(chaseTagCommand);
    // m_controller.x().whileTrue(pathFollowCommand);
    m_controller.y().whileTrue(balanceCommand);
    if (!mrKeith) {
      m_controller.povDown().whileTrue(new ShoulderCommand(m_armSubsystem, 0));
      m_controller.povUp().whileTrue(new ShoulderCommand(m_armSubsystem, 0.95));

      m_controller.povLeft().whileTrue(new ExtenderCommand(m_armSubsystem, 0));
      m_controller.povRight().whileTrue(new ExtenderCommand(m_armSubsystem, -9));

      m_controller.b().whileTrue(gamePiecePlacementCommand);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
