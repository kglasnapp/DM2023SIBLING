// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.swervedrivespecialties.swervelib.SwerveModuleFactory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ChangeNormalModeCommand;
import frc.robot.commands.ChangeTurboModeCommand;
import frc.robot.commands.ConeAlignCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.GrabberDefaultCommand;
import frc.robot.commands.KeyPadStateCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.StraightPathCommand;
import frc.robot.commands.ZeroExtenderCommand;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.commands.ZeroShoulderCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import frc.robot.subsystems.PoseEstimatorAggregator;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.FieldConstants.Community;
import frc.robot.subsystems.FieldConstants.StagingLocations;
import frc.robot.utilities.AutonomousCommandFactory;
import frc.robot.utilities.KeyPadPositionSupplier;
import frc.robot.utilities.PiecePickerPoseProvider;
import frc.robot.utilities.SwerveModule;

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
  private final static CommandXboxController m_controller2 = new CommandXboxController(3);
  // private final static ControllerAvg controllerAverage = new
  // ControllerAvg(m_controller);
  private GenericHID keyPadController = new GenericHID(1);

  private Transform3d competitionCamerasTransform[] = {
    new Transform3d(new Translation3d(0, 0.14, 0), new Rotation3d()),
    new Transform3d(new Translation3d(0, -0.14, 0), new Rotation3d())
  };

  private Transform3d siblingCamerasTransform[] = {
    new Transform3d(new Translation3d(0, 0.14, 0), new Rotation3d()),
    new Transform3d(new Translation3d(0, -0.14, 0), new Rotation3d())
  };

  DigitalInput digitalInput = new DigitalInput(0);

  private final PoseEstimatorAggregator poseEstimator = new PoseEstimatorAggregator(new PoseEstimatorSubsystem[] {
      new PoseEstimatorSubsystem("1", new PhotonCamera("gloworm1"),
          digitalInput.get()?siblingCamerasTransform[0]:competitionCamerasTransform[0], m_drivetrainSubsystem),
      new PoseEstimatorSubsystem("2", new PhotonCamera("gloworm2"),
          digitalInput.get()?siblingCamerasTransform[1]:competitionCamerasTransform[1], m_drivetrainSubsystem),
  });

  private final PiecePickerPoseProvider pickerPoseProvider = new PiecePickerPoseProvider();
  private final ConeAlignCommand coneAlignCommand = new ConeAlignCommand(pickerPoseProvider, m_drivetrainSubsystem,
      poseEstimator);
  private final BalanceCommand balanceCommand = new BalanceCommand(m_drivetrainSubsystem);

  public static boolean mrKeith = true;
  private SlewRateLimiter sLX = new SlewRateLimiter(9);
  private SlewRateLimiter sLY = new SlewRateLimiter(9);
  private SlewRateLimiter sRX = new SlewRateLimiter(1);
  public static SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // A chooser for autonomous commands
    autonomousChooser.setDefaultOption("Simple Case",
        AutonomousCommandFactory.getAutonomousSimpleCommand(m_drivetrainSubsystem, m_armSubsystem, grabberSubsystem)
        .andThen(AutonomousCommandFactory.getSetPositionAndBalanceCommand(m_drivetrainSubsystem, poseEstimator))
      );
    autonomousChooser.addOption("Simple Case and Left out",
        AutonomousCommandFactory.getAutonomousSimpleAndLeftOutCommand(m_drivetrainSubsystem, m_armSubsystem,
            grabberSubsystem));
    autonomousChooser.addOption("Simple Case and Right out",
        AutonomousCommandFactory.getAutonomousSimpleAndRightOutCommand(m_drivetrainSubsystem, m_armSubsystem,
            grabberSubsystem));

    // Add commands to the autonomous command chooser
    autonomousChooser.addOption("Case 1  left",
        getAutonomousCommandCase1(0).andThen(new StraightPathCommand(m_drivetrainSubsystem,
            getPoseEstimatorForTarget(poseEstimator, 2),
            getFinalPoseForCase1(0))));

    autonomousChooser.addOption("Case 1  middle", getAutonomousCommandCase1(1));
    autonomousChooser.addOption("CaSe 1  right", getAutonomousCommandCase1(2)
        .andThen(new StraightPathCommand(m_drivetrainSubsystem, getPoseEstimatorForTarget(poseEstimator, 0),
            getFinalPoseForCase1(2))));

    // autonomousChooser.addOption("Case 2 left", getAutonomousCommandCase2(0));
    // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
    autonomousChooser.addOption("Case 2 red", getAutonomousCommandCase2Red());
    autonomousChooser.addOption("Case 2 blue", getAutonomousCommandCase2Blue());
    // autonomousChooser.addOption("Case 2 right", getAutonomousCommandCase2(2));
    // autonomousChooser.addOption("Case 3", getAutonomousCommandCase3());

    // Put the chooser on the dashboard
    SmartDashboard.putData("Autonomous Mode", (Sendable) autonomousChooser);

    // The robot's subsystems and commands are defined here...
    grabberSubsystem.setDefaultCommand(new GrabberDefaultCommand(grabberSubsystem,
        m_controller.b(),
        m_controller.a(),
        m_controller.povDown(),
        m_controller2.povRight(),
        m_controller2.povLeft(),
        m_controller2.povDown()));

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // m_drivetrainSubsystem,
    // () -> (SwerveModuleFactory.powerRatio == 1 ?
    // -modifyAxis(squareWithSign(sLY.calculate(m_controller.getLeftY())))
    // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
    // : -modifyAxis(squareWithSign(m_controller.getLeftY()))
    // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
    // () -> (SwerveModuleFactory.powerRatio == 1 ?
    // -modifyAxis(squareWithSign(sLX.calculate(m_controller.getLeftX())))
    // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
    // : -modifyAxis(squareWithSign(m_controller.getLeftX()))
    // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
    // () -> (SwerveModuleFactory.powerRatio == 1
    // ? -modifyAxis(squareWithSign(sRX.calculate(m_controller.getRightX())))
    // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
    // : -modifyAxis(squareWithSign(m_controller.getRightX()))
    // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
    // m_controller));

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> (SwerveModule.powerRatio == 1 ? -modifyAxis((sLY.calculate(m_controller.getLeftY())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((m_controller.getLeftY()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModule.powerRatio == 1 ? -modifyAxis((sLX.calculate(m_controller.getLeftX())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((m_controller.getLeftX()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModule.powerRatio == 1
            ? -modifyAxis((sRX.calculate(m_controller.getRightX())))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((m_controller.getRightX()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        m_controller2.leftBumper()));

    // TODO: to change the turbo, change the x() above
    m_armSubsystem.setDefaultCommand(new DefaultArmCommand(m_armSubsystem,
        () -> (RobotContainer.getLeftBumper() ? -1 : 1) * RobotContainer.getLeftTrigger(),
        () -> (RobotContainer.getRightBumper() ? -1 : 1) * RobotContainer.getRightTrigger(),
        () -> RobotContainer.getLeftTrigger2(),
        () -> (RobotContainer.getRightBumper2() ? -1 : 1) * RobotContainer.getRightTrigger2()));
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

  public static double getRightTrigger2() {
    return m_controller2.getRightTriggerAxis();
  }

  public static double getLeftTrigger2() {
    return m_controller2.getLeftTriggerAxis();
  }

  public static boolean getRightBumper2() {
    return m_controller2.getHID().getRawButton(6);
  }

  public static boolean getLeftBumper2() {
    return m_controller2.getHID().getRawButton(5);
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

    m_controller.x().whileTrue(
        getAutonomousCommandCase2Blue()

    );

    m_controller.y().whileTrue(new RotateCommand(m_drivetrainSubsystem));
      //new ZeroGyroCommand(m_drivetrainSubsystem, balanceCommand, 180));
    // balanceCommand.andThen(new PrintCommand("Finished Balancing")));

    // todo: cone align
    m_controller2.povUp().whileTrue(coneAlignCommand.andThen(new ShoulderCommand(m_armSubsystem, 60352)
        .andThen(new GrabberCommand(grabberSubsystem, true))

        .andThen(new WaitCommand(1))
        .andThen(new ShoulderCommand(m_armSubsystem, 37352))
        .andThen(new ExtenderCommand(m_armSubsystem, 91000))));

    m_controller2.y().whileTrue(AutonomousCommandFactory.pickupCubeCommand(m_drivetrainSubsystem, 
      m_armSubsystem, grabberSubsystem, poseEstimator));
      
    //AutonomousCommandFactory.getSetPositionAndBalanceCommand(m_drivetrainSubsystem, poseEstimator));
    // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
    // new Pose2d( poseEstimator.get().getX(), poseEstimator.get().getY(), new
    // Rotation2d(Math.toRadians(160))))
    // .andThen(new BalanceCommand(m_drivetrainSubsystem)));
    // controller 2 a = zero the arm
    m_controller2.a()
        .whileTrue(new ZeroExtenderCommand(m_armSubsystem).andThen(new ZeroShoulderCommand(m_armSubsystem)));
    // controller 2 y = pick up from floor position
    // m_controller2.y().whileTrue(new ShoulderCommand(m_armSubsystem, 60352)
    // .andThen(new GrabberCommand(grabberSubsystem, true))
    // .andThen(new WaitCommand(1))
    // .andThen(new ShoulderCommand(m_armSubsystem, 40352))
    // .andThen(new ExtenderCommand(m_armSubsystem, 183023 * 16 / 36)));

    // m_controller2.b().whileTrue(getAutonomousCommandCase3());
    m_controller2.b().whileTrue(new ShoulderCommand(m_armSubsystem, 245000));

    // m_controller2.x().whileTrue(new StraightPathCommand(m_drivetrainSubsystem,
    // poseEstimator,
    // new Pose2d(2.29, 7, new Rotation2d(Math.toRadians(180)))));
    // new ShouldeCommand(m_armSubsystem, 241000));
    m_controller2.x()
        .whileTrue(new GrabberCommand(grabberSubsystem, false).andThen(new ShoulderCommand(m_armSubsystem, 195500)));

    m_controller.povRight().whileTrue(pickUpFromRight());
    m_controller.povLeft().whileTrue(pickUpFromLeft());
    for (int i = 0; i < 9; ++i) {
      getKeyPadControllerButton(i).whileTrue(getCommandFor(i));
    }
    for (int i = 0; i < 3; ++i) {
      getKeyPadControllerButton(9 + i).onTrue(new KeyPadStateCommand(i));
    }
  }

  public CommandBase pickUpFromLeft() {
    return pickUpFrom(getPickupPose(false, 0), getPickupPose(false, 1));
  }

  public CommandBase pickUpFromRight() {
    return pickUpFrom(getPickupPose(true, 0), getPickupPose(true, 1));
  }

  public CommandBase pickUpFrom(Pose2d pose2d1, Pose2d pose2d2) {

    return new CommandBase() {
      @Override
      public void initialize() {
        DefaultDriveCommand.autonomous = true;
        SwerveModule.powerRatio = 3.5;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    }.andThen(
        Commands.parallel(new StraightPathCommand(m_drivetrainSubsystem,
            poseEstimator,
            pose2d1),
            new ShoulderCommand(m_armSubsystem, 195000),
            new GrabberCommand(grabberSubsystem, false))
            .andThen(
                new ExtenderCommand(m_armSubsystem, 186000))
            .andThen(new WaitCommand(0.25))
            .andThen(new ShoulderCommand(m_armSubsystem, 201000))
            .andThen(new ZeroExtenderCommand(m_armSubsystem))
            .andThen(new ShoulderCommand(m_armSubsystem, 0))
            .finallyDo(new BooleanConsumer() {
              public void accept(boolean value) {
                DefaultDriveCommand.autonomous = false;
                SwerveModule.powerRatio = SwerveModule.TURBO;
              }
            }));
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

  CommandBase getCommandFor(int pos) {
    double extenderGoals[] = new double[] {
        190000, 45000, 0
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
        // new double[] { 191432 * 1.1, 87133 * 2.08 * 1.1, 73000 * 1.1}, // used to
        // adjust for forward bend
        // new double[] { 150000 * 1.1, 66133 * 2.08 * 1.1, 73000 * 1.1},
        new double[] { 250000, 185000, 67000 }, // this is the old one - pre bent
        new double[] { 215000, 138000, 67000 },
    };
    return new CommandBase() {
      @Override
      public void initialize() {
        DefaultDriveCommand.autonomous = true;
        SwerveModule.powerRatio = SwerveModule.NORMAL;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    }.andThen(
        Commands.parallel(
            new PrintCommand("Going to coordinate " + new KeyPadPositionSupplier(pos).get()),
            new StraightPathCommand(m_drivetrainSubsystem,
                getPoseEstimatorForTarget(poseEstimator, pos),
                new KeyPadPositionSupplier(pos)),
            new ShoulderCommand(m_armSubsystem, shoulderGoals[0][pos / 3]))
            .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0.2, 0, 0, 1000))
            .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0))
            .andThen(new ExtenderCommand(m_armSubsystem, extenderGoals[pos / 3]))
            .andThen(new ShoulderCommand(m_armSubsystem, shoulderGoals[1][pos / 3])))
        .finallyDo(new BooleanConsumer() {
          public void accept(boolean value) {
            DefaultDriveCommand.autonomous = false;
          }
        });

  }

  /**
   * This method returns the pose provider to use depending on which target are we
   * going after.
   * If the robot is going after a cone post on the left, we use the pose
   * estimator with the camera on the right.
   * IF the robot is going after a cube (center) we use the aggregator of both
   * cameras.
   * If the robot is going after a cone post on the right, we use the pose
   * estimator with the camera on the left.
   * pos is the number pressed in the keypad.
   * 0 to 2 in the first row
   * 3 to 5 in the second
   * 6 to 8 in the last one.
   * 0 % 3 = 0
   * 3 % 3 = 0
   * 6 % 3 = 0
   * and the rest accordingly.
   * 
   * @param poseEstimatorAggregator
   * @param pos
   * @return
   */
  Supplier<Pose2d> getPoseEstimatorForTarget(PoseEstimatorAggregator poseEstimatorAggregator, int pos) {
    return poseEstimatorAggregator;
    // pos is in the left, we use camera on the right
    // if (pos % 3 == 2) {
    // System.out.println("using camera 2");
    // return poseEstimatorAggregator.poseEstimators[1]::getCurrentPose;
    // }
    // // pos is in the center, we use both cameras
    // if (pos % 3 == 1) {
    // System.out.println("using both cameras");
    // return poseEstimator.poseEstimators[1]::getCurrentPose;
    // // return poseEstimatorAggregator;
    // }
    // // we use the caemra on the left.
    // System.out.println("using camera 1");
    // return poseEstimator.poseEstimators[1]::getCurrentPose;
  }

  CommandBase getCommandForAutonomous(int pos) {
    double extenderGoals[] = new double[] {
        1250000, 45000, 0
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
        new double[] { 250000, 185000, 67000 },
        new double[] { 215000, 138000, 67000 },
    };
    return new CommandBase() {
      @Override
      public void initialize() {
        DefaultDriveCommand.autonomous = true;
        SwerveModule.powerRatio = SwerveModule.NORMAL;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    }.andThen(
        Commands.parallel(new StraightPathCommand(m_drivetrainSubsystem,
            getPoseEstimatorForTarget(poseEstimator, pos),
            new KeyPadPositionSupplier(pos)),
            new ShoulderCommand(m_armSubsystem, shoulderGoals[0][pos / 3]))
            .andThen(new DriveCommand(m_drivetrainSubsystem, 0.2, 0, 0))
            .andThen(new WaitCommand(1))
            .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0))
            .andThen(new ExtenderCommand(m_armSubsystem, extenderGoals[pos / 3]))
            .andThen(new ShoulderCommand(m_armSubsystem, shoulderGoals[1][pos / 3])))
        .finallyDo(new BooleanConsumer() {
          public void accept(boolean value) {
            DefaultDriveCommand.autonomous = false;
          }
        });

  }

  Trigger getKeyPadControllerButton(int buttonId) {
    return keyPadController.button(buttonId + 1, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  /**
   * the state can be 0, 1, or 2. It means the position where the robot is setup
   * at start
   * left, middle or right.
   * red alliance second peice locations from 0,0 :
   * 1. 3.44 + 1.22(0), 5.3
   * 2. 3.44 + 1.22(1), 5.3
   * 3. 3.44 + 1.22(2), 5.3
   * 4. 3.44 + 1.22(3), 5.3
   * blue second peice locaitons from 0,0:
   * 1. 0.92 + 1.22(0), 5.3
   * 2. 0.92 + 1.22(1), 5.3
   * 3. 0.92 + 1.22(2), 5.3
   * 4. 0.92 + 1.22(3), 5.3
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandCase1(int state) {
    int pos = 0;
    if (DriverStation.getAlliance() == Alliance.Red) {
      pos = 2;
    }
    Command command = new CommandBase() {
      @Override
      public void initialize() {
        KeyPadPositionSupplier.state = state;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    }.andThen(
        getCommandForAutonomous(pos))
        .andThen(new ZeroGyroCommand(m_drivetrainSubsystem, balanceCommand, (180)))
        .andThen(new ZeroGyroCommand(m_drivetrainSubsystem, balanceCommand, (180)))
        .andThen(new GrabberCommand(grabberSubsystem, true))
        .andThen(new WaitCommand(2))
        .andThen(new ZeroExtenderCommand(m_armSubsystem))
        .andThen(new ShoulderCommand(m_armSubsystem, 0));
    command.setName("Case 1");
    return command;
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

  public Command getAutonomousCommandCase2Red() {
    CommandBase command = new ZeroGyroCommand(m_drivetrainSubsystem, balanceCommand, (180))
        .andThen(new GrabberCommand(grabberSubsystem, false))
        .andThen(new KeyPadStateCommand(1))
        .andThen(getCommandFor(1))
        .andThen(new GrabberCommand(grabberSubsystem, true))
        .andThen(new WaitCommand(0.5))
        .andThen(new ZeroExtenderCommand(m_armSubsystem))
        .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
            new Pose2d(new Translation2d(2.1, 5.24), new Rotation2d(Math.toRadians(180)))))
        .andThen(new ZeroShoulderCommand(m_armSubsystem))
        .andThen(new ChangeTurboModeCommand())
        .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        .andThen(new WaitCommand(0.5))
        .andThen(new ChangeNormalModeCommand())
        .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        .andThen(new WaitCommand(2.5))
        // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
        // .andThen(new WaitCommand(2))
        .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
    // .andThen(new BalanceCommand(m_drivetrainSubsystem))
    command.setName("Case 2 red");
    return command;
  }

  public Command getAutonomousCommandCase2Blue() {
    CommandBase command = new ZeroGyroCommand(m_drivetrainSubsystem, balanceCommand, (180))
        .andThen(new GrabberCommand(grabberSubsystem, false))
        .andThen(new KeyPadStateCommand(1))
        .andThen(getCommandFor(1))
        .andThen(new GrabberCommand(grabberSubsystem, true))
        .andThen(new WaitCommand(0.5))
        .andThen(new ZeroExtenderCommand(m_armSubsystem))
        .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
            new Pose2d(new Translation2d(2.1, KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
                new Rotation2d(Math.toRadians(180)))))
        .andThen(new ZeroShoulderCommand(m_armSubsystem))
        .andThen(new ChangeTurboModeCommand())
        .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        .andThen(new WaitCommand(0.5))
        .andThen(new ChangeNormalModeCommand())
        .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        .andThen(new WaitCommand(2.5))
        // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
        // .andThen(new WaitCommand(2))
        .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
    // .andThen(new BalanceCommand(m_drivetrainSubsystem))
    command.setName("Case 2 blue");
    return command;
  }

  public Command getAutonomousCommandCase3() {
    KeyPadPositionSupplier.state = 0;
    Command command = new CommandBase() {
      @Override
      public void initialize() {
        KeyPadPositionSupplier.state = 0;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    }.andThen(getCommandFor(0)
        .andThen(new GrabberCommand(grabberSubsystem, true))
        // .andThen(new WaitCommand(1))
        .andThen(new ZeroExtenderCommand(m_armSubsystem))
        // .andThen(
        // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // new Pose2d(5.75,
        // 7.4,
        // new Rotation2d(Math.toRadians(180)))))
        .andThen(
            new StraightPathCommand(m_drivetrainSubsystem, getPoseEstimatorForTarget(poseEstimator, 0),
                new Pose2d(4.4,
                    5.3,
                    new Rotation2d(Math.toRadians(0)))))
        .andThen(
            new StraightPathCommand(m_drivetrainSubsystem, getPoseEstimatorForTarget(poseEstimator, 0),
                new Pose2d(6.4,
                    5.3,
                    new Rotation2d(Math.toRadians(0)))))
        .andThen(new ShoulderCommand(m_armSubsystem, 40352))
        .andThen(new ExtenderCommand(m_armSubsystem, 183023 * 16 / 36))
        .andThen(new GrabberCommand(grabberSubsystem, false))
        .andThen(new GrabberCommand(grabberSubsystem, false))
        .andThen(new WaitCommand(2))
        .andThen(new ZeroExtenderCommand(m_armSubsystem))
        .andThen(Commands.parallel(
            new ShoulderCommand(m_armSubsystem, 190432),
            new StraightPathCommand(m_drivetrainSubsystem, getPoseEstimatorForTarget(poseEstimator, 0),
                new Pose2d(3.2, 5.3,
                    new Rotation2d(Math.toRadians(0))))))
        .andThen(getCommandFor(4))
        .andThen(new GrabberCommand(grabberSubsystem, true)));
    command.setName("case 3");
    return command;
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
