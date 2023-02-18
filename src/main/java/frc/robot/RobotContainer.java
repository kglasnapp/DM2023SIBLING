// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;

import com.swervedrivespecialties.swervelib.SwerveModuleFactory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ConeAlignCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.GrabberDefaultCommand;
import frc.robot.commands.KeyPadStateCommand;

import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.StraightPathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import frc.robot.subsystems.PoseEstimatorAggregator;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.utilities.KeyPadPositionSupplier;
import frc.robot.utilities.PiecePickerPoseProvider;

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
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // private final XboxController m_controller = new XboxController(2);
  private final static CommandXboxController m_controller = new CommandXboxController(2);
  //private final static ControllerAvg controllerAverage = new ControllerAvg(m_controller);
  private GenericHID keyPadController = new GenericHID(1);

  private final PoseEstimatorAggregator poseEstimator = new PoseEstimatorAggregator(new PoseEstimatorSubsystem[] {
      new PoseEstimatorSubsystem("1", new PhotonCamera("gloworm1"),
          new Transform3d(new Translation3d(0, 0.14, 0), new Rotation3d()), m_drivetrainSubsystem),
      new PoseEstimatorSubsystem("2", new PhotonCamera("gloworm2"),
          new Transform3d(new Translation3d(0, -0.14, 0), new Rotation3d()), m_drivetrainSubsystem),
          
  });

  private final PiecePickerPoseProvider pickerPoseProvider = new PiecePickerPoseProvider();
  private final ConeAlignCommand coneAlignCommand = new ConeAlignCommand(pickerPoseProvider, m_drivetrainSubsystem,poseEstimator);
  private final BalanceCommand balanceCommand = new BalanceCommand(m_drivetrainSubsystem);

  public static boolean mrKeith = true;
  private SlewRateLimiter sLX = new SlewRateLimiter(9);
  private SlewRateLimiter sLY = new SlewRateLimiter(9);
  private SlewRateLimiter sRX = new SlewRateLimiter(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // The robot's subsystems and commands are defined here...
    grabberSubsystem.setDefaultCommand(new GrabberDefaultCommand(grabberSubsystem,
        m_controller.povRight(),
        m_controller.povLeft(),
        m_controller.povDown()));

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // m_drivetrainSubsystem,
    // () -> -modifyAxis(squareWithSign(m_controller.getLeftY())) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(squareWithSign( m_controller.getLeftX())) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(squareWithSign(m_controller.getRightX()))
    // * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> (SwerveModuleFactory.powerRatio == 1 ? -modifyAxis(squareWithSign(sLY.calculate(m_controller.getLeftY())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis(squareWithSign(m_controller.getLeftY()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModuleFactory.powerRatio == 1 ? -modifyAxis(squareWithSign(sLX.calculate(m_controller.getLeftX())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis(squareWithSign(m_controller.getLeftX()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModuleFactory.powerRatio == 1 ? -modifyAxis(squareWithSign(sRX.calculate(m_controller.getRightX())))
        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
        : -modifyAxis(squareWithSign(m_controller.getRightX()))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        m_controller));
        // TODO: to change the turbo, change the x() above
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
    // m_controller.b().whileTrue(gamePiecePlacementCommand);
    
    // todo: cone align    
    m_controller.a().whileTrue(coneAlignCommand);

    // pick up from double substation

    m_controller.x().whileTrue(
        new StraightPathCommand(m_drivetrainSubsystem,
            poseEstimator, 
            getPickupPose(false,0))
            .andThen(new ShoulderCommand(m_armSubsystem, 165000))
            .andThen(new GrabberCommand(grabberSubsystem, true))
            .andThen(new PrintCommand("finished grab open hand"))
            .andThen(new WaitCommand(0.5))

            .andThen(new PrintCommand("Finished waiting"))
            .andThen(
              new ExtenderCommand(m_armSubsystem, 245000))

            .andThen(new GrabberCommand(grabberSubsystem, false))
            .andThen(new WaitCommand(1.5))
            .andThen(new GrabberCommand(grabberSubsystem, false))
            .andThen(new WaitCommand(0.1))
            .andThen(new GrabberCommand(grabberSubsystem, false))
            .andThen(new ShoulderCommand(m_armSubsystem, 201000))
            .andThen(new ExtenderCommand(m_armSubsystem, 0))
            .andThen(new StraightPathCommand(m_drivetrainSubsystem,
            poseEstimator,
            getPickupPose(false,1)))
            .andThen(new ShoulderCommand(m_armSubsystem, 0)));
            m_controller.b().whileTrue(
              new StraightPathCommand(m_drivetrainSubsystem,
                  poseEstimator, 
                  getPickupPose(true,0))
                  .andThen(new ShoulderCommand(m_armSubsystem, 165000))
                  .andThen(new GrabberCommand(grabberSubsystem, true))
                  .andThen(new PrintCommand("finished grab open hand"))
                  .andThen(new WaitCommand(0.5))
      
                  .andThen(new PrintCommand("Finished waiting"))
                  .andThen(
                    new ExtenderCommand(m_armSubsystem, 245000))
      
                  .andThen(new GrabberCommand(grabberSubsystem, false))
                  .andThen(new WaitCommand(1.5))
                  .andThen(new GrabberCommand(grabberSubsystem, false))
                  .andThen(new WaitCommand(0.1))
                  .andThen(new GrabberCommand(grabberSubsystem, false))
                  .andThen(new ShoulderCommand(m_armSubsystem, 201000))
                  .andThen(new ExtenderCommand(m_armSubsystem, 0))
                  .andThen(new StraightPathCommand(m_drivetrainSubsystem,
                  poseEstimator,
                  getPickupPose(true,1)))
                  .andThen(new ShoulderCommand(m_armSubsystem, 0)));
    for (int i = 0; i < 9; ++i) {
      getKeyPadControllerButton(i).whileTrue(getCommandFor(i));
    }
    for (int i = 0; i < 3; ++i) {
      getKeyPadControllerButton(9 + i).onTrue(new KeyPadStateCommand(i));
    }
  }

  public Pose2d getPickupPose(boolean right, int phase) {
    var alliance = DriverStation.getAlliance();
    Pose2d pose = null;
    if (right) {
      pose = new Pose2d(new Translation2d(15.14, 0.55), new Rotation2d(Math.toRadians(0)));
    } else {
      pose = new Pose2d(new Translation2d(15.14, 1.95), new Rotation2d(Math.toRadians(0)));
    }
    if (phase == 1) {
      pose = new Pose2d(pose.getX()-0.36, pose.getY(), pose.getRotation());
    }
    if (alliance == Alliance.Blue) {
        pose = new Pose2d(pose.getX(), KeyPadPositionSupplier.FIELD_WIDTH - pose.getY(), pose.getRotation());
    }
    return pose;
}

  CommandBase getCommandFor(int pos) {
    double extenderGoals[] = new double[] {
        468000, 140354, 0
    };
    /**
     * The shoulder command goes up, then the extender command goes, after that the
     * shoulder command goes down.
     * Going up is considered: phase 0.
     * Going down after the extender command is: phase 1.
     * Those two phases have a target pos that the shoulder encoder needs to
     * achieve.
     * Those goals are configured in this array.
     * phase0: up, middle, floor
     * phase1: up, middle, floor
     */
    double shoulderGoals[][] = new double[][] {
        new double[] { 104054 * 2.08, 87133 * 2.08, 87133 * 2.08 },
        new double[] { 87133 * 2.08, 66133 * 2.08, 20000 * 2.08 },
    };
    return new StraightPathCommand(m_drivetrainSubsystem,
        poseEstimator, 
        new KeyPadPositionSupplier(pos))
        .andThen(new ShoulderCommand(m_armSubsystem, shoulderGoals[0][pos / 3]))
        .andThen(new ExtenderCommand(m_armSubsystem, extenderGoals[pos / 3]))
        .andThen(new ShoulderCommand(m_armSubsystem, shoulderGoals[1][pos / 3]));
  }

  Trigger getKeyPadControllerButton(int buttonId) {
    return keyPadController.button(buttonId + 1, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
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
