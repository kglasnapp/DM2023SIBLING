
package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.OperatorButtons;
import frc.robot.RobotContainer.RobotMode;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DisplayLogCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.State;
import frc.robot.commands.PositionCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.commands.RobotOrientedDriveDeacceleratedCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.SetModeConeCube;
import frc.robot.commands.StraightPathCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightPoseSubsystem;
import frc.robot.utilities.KeyPadPositionSupplier;
import frc.robot.utilities.SwerveModule;

public class Autonomous {
    DrivetrainSubsystem drivetrainSubsystem;
    RobotContainer robotContainer;
    private final BalanceCommand balanceCommand;

    public static Command test;

    public Autonomous(RobotContainer robotContainer, DrivetrainSubsystem drivetrain, IntakeSubsystem intake) {
        this.robotContainer = robotContainer;
        this.drivetrainSubsystem = drivetrain;
        balanceCommand = new BalanceCommand(drivetrain);
        RobotContainer.autonomousChooser.setDefaultOption("Case 1 outside", getAutonomousCommandCase1());
        RobotContainer.autonomousChooser.addOption("Case left turn", caseCommand("Left Turn", intake, 0.2));
        RobotContainer.autonomousChooser.addOption("Case right turn", caseCommand("Right Turn", intake, -0.6));
        RobotContainer.autonomousChooser.addOption("Case 3 Center Balance", getAutonomousCommandCase3());
        RobotContainer.autonomousChooser.addOption("Case 4 Center Pickup Balance", getAutonomousCommandCase4());
        RobotContainer.autonomousChooser.addOption("Test Movements", testMovements(drivetrain, intake, robotContainer));
        RobotContainer.autonomousChooser.addOption("Balance ", getOverAndBalanceCommand(drivetrain));

        RobotContainer.autonomousChooser.addOption("Blue 8",
                Autonomous.get2PiecesCommand(robotContainer,
                        "/home/lvuser/deploy/Blue8.wpilib.json",
                        "/home/lvuser/deploy/Blue8Return.wpilib.json",
                        new Pose2d(6.83, 0.9, new Rotation2d(Math.toRadians(0)))));

        test = testMovements(drivetrain, intake, robotContainer);
        // Put the chooser on the dashboard
        SmartDashboard.putData("Autonomous Mode", RobotContainer.autonomousChooser);
        //SmartDashboard.putData("Autonomous Mode", autonomousChooser);

    }

    private Command getAutonomousCommandCase1() {
        CommandBase command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))
                .andThen(new WaitCommand(0.5))
                .andThen(new DisplayLogCommand("Run Case 1"));
        return command;
    }

    public Command getAutonomousCommandCase2() {
        CommandBase command = new DisplayLogCommand("Case 2")
                .andThen(new WaitCommand(0.5));
        return command;
    }

    private Command getAutonomousCommandCase3() {
        CommandBase command = new DisplayLogCommand("Case 3")
                .andThen(new WaitCommand(0.5));
        return command;
    }

    private Command getAutonomousCommandCase4() {
        CommandBase command = new DisplayLogCommand("Case 4")
                .andThen(new WaitCommand(0.5));
        return command;
    }

    private Command caseCommand(String name, IntakeSubsystem intakeSubsystem, double deltaY) {
        return new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))
                .andThen(new SetModeConeCube(RobotMode.Cube))
                //.andThen(new PositionCommand(this, OperatorButtons.LOW))
                .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300))
                // .andThen(new PositionCommand(this, OperatorButtons.HOME))
                .andThen(
                        new StraightPathCommand(drivetrainSubsystem,
                                robotContainer.limeLightPoseSubsystem,
                                new Pose2d(4.49, 5.08, new Rotation2d(Math.toRadians(180)))))
                .andThen(new RotateCommand(drivetrainSubsystem))
                .andThen(new StraightPathCommand(drivetrainSubsystem, robotContainer.limeLightPoseSubsystem,
                        new Pose2d(4.79, 5.08, new Rotation2d(Math.toRadians(0)))))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
                .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.IN, 1000))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
                .andThen(new RotateCommand(drivetrainSubsystem))
                .andThen(new StraightPathCommand(drivetrainSubsystem,
                        robotContainer.limeLightPoseSubsystem,
                        new Pose2d(1.89, 4.88, new Rotation2d(Math.toRadians(180)))))
                .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300));
    }

    private Command testMovements(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intake,
            RobotContainer robotContainer) {
        return new DisplayLogCommand("Test Case")
                //.andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, .4, 0, 0, 500))
                .andThen(new SetModeConeCube(RobotMode.Cube))
                //.andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, 0, 0, 0, 50))
                //.andThen(new PositionCommand(robotContainer, OperatorButtons.LOW))
                .andThen(new IntakeCommand(intake, State.OUT, 1000))
                .andThen(new WaitCommand(.5))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
                .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, -.3, 0, 0, 1000))
                .andThen(new WaitCommand(.25))
                .andThen(new RotateCommand(drivetrainSubsystem))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
                .andThen(new IntakeCommand(intake, State.IN, 1000))
                .andThen(new RobotOrientedDriveDeacceleratedCommand(drivetrainSubsystem, -.3, 0, 0, 300));
        // .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, 0, 0, 0, 50));
    }

    private Supplier<Pose2d> poseEstimator;

    public Command caseCommandOld(String name, IntakeSubsystem intakeSubsystem, double speedY, int durationY) {
        CommandBase command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))
                .andThen(new SetModeConeCube(RobotMode.Cube))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.HIGH))
                .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 1000))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
                .andThen(new RobotOrientedDriveDeacceleratedCommand(drivetrainSubsystem, 0, speedY, 0, durationY))
                .andThen(new WaitCommand(0.1))
                .andThen(new RobotOrientedDriveDeacceleratedCommand(drivetrainSubsystem, -.7, 0, 0, 1500))
                .andThen(new RotateCommand(drivetrainSubsystem))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
                .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.IN, 1000));
        command.setName(name);
        return command;
    }

    public Command test2() {
        CommandBase command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))
                .andThen(new StraightPathCommand(drivetrainSubsystem, poseEstimator,
                        new Pose2d(new Translation2d(2.1, KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
                                new Rotation2d(Math.toRadians(180)))))
                .andThen(new DriveCommand(drivetrainSubsystem, -2, 0, 100))
                .andThen(new WaitCommand(0.5))
                .andThen(new DriveCommand(drivetrainSubsystem, -2, 0, 0))
                .andThen(new WaitCommand(2.5))
                .andThen(new DriveCommand(drivetrainSubsystem, -0.05, 0, 0))
                .andThen(new WaitCommand(2))
                .andThen(new DriveCommand(drivetrainSubsystem, 0, 0, 0))
                .andThen(new BalanceCommand(drivetrainSubsystem));
        command.setName("Test 2");
        return command;
    }

    public static Command getOverAndBalanceCommand(DrivetrainSubsystem m_drivetrainSubsystem) {
        BalanceCommand balanceCommand = new BalanceCommand(m_drivetrainSubsystem);

        return new CommandBase() {
            @Override
            public void initialize() {
                DefaultDriveCommand.autonomous = true;
                SwerveModule.setPowerRatio(1.5);
                balanceCommand.zeroGyroscope();
            }

            @Override
            public boolean isFinished() {
                return true;
            }

        }
                .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.03, 0, 0, 3500))
                .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0.02, 0, 0, 2500))
                .andThen(balanceCommand);
    }

    public static Command get2PiecesCommand(
            RobotContainer robotContainer,
            String splineGetCube, String splineDropCube, Pose2d cubePose) {
        DrivetrainSubsystem drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        BalanceCommand balanceCommand = robotContainer.balanceCommand;
        LimeLightPoseSubsystem limeLightPoseSubsystem = robotContainer.limeLightPoseSubsystem;
        IntakeSubsystem intakeSubsystem = robotContainer.intakeSubsystem;

        return new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, 180)
                .andThen(new CommandBase() {
                    @Override
                    public void initialize() {

                        String pipeLine = "botpose_wpiblue";//(Robot.alliance == Alliance.Red) ? "botpose_wpired" : "botpose_wpiblue";
                        double llPose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry(pipeLine)
                                .getDoubleArray(new double[6]);
                        double cameraAngle = Math.toRadians(llPose[5]);
                        Pose2d visionPose = new Pose2d(llPose[0], llPose[1], new Rotation2d(cameraAngle));
                        limeLightPoseSubsystem.setCurrentPose(visionPose);
                        // limeLightPoseSubsystem.setCurrentPose(new Pose2d(1.89, 0.5, new Rotation2d(Math.toRadians(180))));
                    }

                    @Override
                    public boolean isFinished() {
                        return true;
                    }
                })
                .andThen(new SetModeConeCube(RobotMode.Cube))
                //.andThen(new PositionCommand(this, OperatorButtons.LOW))
                .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300))
                // .andThen(new PositionCommand(this, OperatorButtons.HOME))
                .andThen(
                        // new StraightPathCommand(drivetrainSubsystem,
                        //     limeLightPoseSubsystem, new Pose2d(5.5, 0.98, new Rotation2d(Math.toRadians(180)))))
                        new TrajectoryCommand(splineGetCube,
                                drivetrainSubsystem, limeLightPoseSubsystem))
                // .andThen(new RotateCommand(drivetrainSubsystem))
                .andThen(grabCube(robotContainer, drivetrainSubsystem, limeLightPoseSubsystem, intakeSubsystem,
                        cubePose))
                .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))

                // .andThen(new RotateCommand(drivetrainSubsystem))
                .andThen(
                        // new StraightPathCommand(drivetrainSubsystem,
                        //   limeLightPoseSubsystem, new Pose2d(1.89, 0.5, new Rotation2d(Math.toRadians(180))))

                        new TrajectoryCommand(splineDropCube,
                                drivetrainSubsystem, limeLightPoseSubsystem))
                .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300));
    }

    static Command grabCube(
            RobotContainer robotContainer,
            DrivetrainSubsystem drivetrainSubsystem,
            LimeLightPoseSubsystem limeLightPoseSubsystem, IntakeSubsystem intakeSubsystem, Pose2d cubePose) {
        return new PositionCommand(robotContainer, OperatorButtons.GROUND)
                .andThen(
                        new IntakeCommand(intakeSubsystem, IntakeCommand.State.IN, 2500)
                                .alongWith(new StraightPathCommand(drivetrainSubsystem,
                                        limeLightPoseSubsystem, cubePose)));
    }

    // autonomousChooser.setDefaultOption("Over and Balance",

    // AutonomousCommandFactory.getAutonomousSimpleLowCommand(m_drivetrainSubsystem,
    // m_armSubsystem, grabberSubsystem)
    // .andThen(AutonomousCommandFactory.getOverAndBalanceCommand(m_drivetrainSubsystem,
    // poseEstimator)));
    // // A chooser for autonomous commands
    // autonomousChooser.setDefaultOption("Middle Balance",
    // AutonomousCommandFactory.getAutonomousSimpleCommand(m_drivetrainSubsystem,
    // m_armSubsystem, grabberSubsystem)
    // .andThen(AutonomousCommandFactory.getSetPositionAndBalanceCommand(m_drivetrainSubsystem,
    // poseEstimator)));
    // autonomousChooser.addOption("Simple Case and Left out",
    // AutonomousCommandFactory.getAutonomousAcceleratedAndLeftOutCommand(m_drivetrainSubsystem,
    // m_armSubsystem,
    // grabberSubsystem));
    // autonomousChooser.addOption("Simple Case and Right out",
    // AutonomousCommandFactory.getAutonomousSimpleAndRightDeacceleratedOutCommand(m_drivetrainSubsystem,
    // m_armSubsystem,
    // grabberSubsystem));

    // // Add commands to the autonomous command chooser
    // autonomousChooser.addOption("Case 1 left",
    // getAutonomousCommandCase1(0).andThen(new
    // StraightPathCommand(m_drivetrainSubsystem,
    // getPoseEstimatorForTarget(poseEstimator, 2),
    // getFinalPoseForCase1(0))));

    // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
    // autonomousChooser.addOption("CaSe 1 right", getAutonomousCommandCase1(2)
    // .andThen(new StraightPathCommand(m_drivetrainSubsystem,
    // getPoseEstimatorForTarget(poseEstimator, 0),
    // getFinalPoseForCase1(2))));

    // autonomousChooser.addOption("Case 2 left", getAutonomousCommandCase2(0));
    // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
    // autonomousChooser.addOption("Case 2 red", getAutonomousCommandCase2Red());
    // autonomousChooser.addOption("Case 2 blue", getAutonomousCommandCase2Blue());
    // autonomousChooser.addOption("Case 2 right", getAutonomousCommandCase2(2));
    // autonomousChooser.addOption("Case 3", getAutonomousCommandCase3());

    public Command getAutonomousCommandCase2Red() {
        CommandBase command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))

                // .andThen(new GrabberCommand(grabberSubsystem, false))
                // .andThen(new KeyPadStateCommand(1))
                // .andThen(getCommandFor(1))
                // .andThen(new GrabberCommand(grabberSubsystem, true))
                // .andThen(new WaitCommand(0.5))
                // .andThen(new ZeroExtenderCommand(m_armSubsystem))
                // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
                // new Pose2d(new Translation2d(2.1, 5.24), new
                // Rotation2d(Math.toRadians(180)))))
                // .andThen(new ZeroShoulderCommand(m_armSubsystem))
                // .andThen(new ChangeTurboModeCommand())
                // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
                .andThen(new WaitCommand(0.5));
        // .andThen(new ChangeNormalModeCommand())
        // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        // .andThen(new WaitCommand(2.5))
        // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
        // // .andThen(new WaitCommand(2))
        // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
        // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
        command.setName("Case 2 red");
        return command;
    }

    // public Command getAutonomousCommandCase2Blue() {
    // CommandBase command = new ZeroGyroCommand(m_drivetrainSubsystem,
    // balanceCommand, (180))
    // .andThen(new GrabberCommand(grabberSubsystem, false))
    // .andThen(new KeyPadStateCommand(1))
    // .andThen(getCommandFor(1))
    // .andThen(new GrabberCommand(grabberSubsystem, true))
    // .andThen(new WaitCommand(0.5))
    // .andThen(new ZeroExtenderCommand(m_armSubsystem))
    // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
    // new Pose2d(new Translation2d(2.1, KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
    // new Rotation2d(Math.toRadians(180)))))
    // .andThen(new ZeroShoulderCommand(m_armSubsystem))
    // .andThen(new ChangeTurboModeCommand())
    // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
    // .andThen(new WaitCommand(0.5))
    // .andThen(new ChangeNormalModeCommand())
    // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
    // .andThen(new WaitCommand(2.5))
    // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
    // // .andThen(new WaitCommand(2))
    // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
    // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
    // command.setName("Case 2 blue");
    // return command;
    // }

    // public Command getAutonomousCommandCase3() {
    // KeyPadPositionSupplier.state = 0;
    // Command command = new CommandBase() {
    // @Override
    // public void initialize() {
    // KeyPadPositionSupplier.state = 0;
    // }

    // @Override
    // public boolean isFinished() {
    // return true;
    // }

    // }.andThen(getCommandFor(0)
    // .andThen(new GrabberCommand(grabberSubsystem, true))
    // // .andThen(new WaitCommand(1))
    // .andThen(new ZeroExtenderCommand(m_armSubsystem))
    // // .andThen(
    // // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
    // // new Pose2d(5.75,
    // // 7.4,
    // // new Rotation2d(Math.toRadians(180)))))
    // .andThen(
    // new StraightPathCommand(m_drivetrainSubsystem,
    // getPoseEstimatorForTarget(poseEstimator, 0),
    // new Pose2d(4.4,
    // 5.3,
    // new Rotation2d(Math.toRadians(0)))))
    // .andThen(
    // new StraightPathCommand(m_drivetrainSubsystem,
    // getPoseEstimatorForTarget(poseEstimator, 0),
    // new Pose2d(6.4,
    // 5.3,
    // new Rotation2d(Math.toRadians(0)))))
    // .andThen(new ShoulderCommand(m_armSubsystem, 40352))
    // .andThen(new ExtenderCommand(m_armSubsystem, 183023 * 16 / 36))
    // .andThen(new GrabberCommand(grabberSubsystem, false))
    // .andThen(new GrabberCommand(grabberSubsystem, false))
    // .andThen(new WaitCommand(2))
    // .andThen(new ZeroExtenderCommand(m_armSubsystem))
    // .andThen(Commands.parallel(
    // new ShoulderCommand(m_armSubsystem, 190432),
    // new StraightPathCommand(m_drivetrainSubsystem,
    // getPoseEstimatorForTarget(poseEstimator, 0),
    // new Pose2d(3.2, 5.3,
    // new Rotation2d(Math.toRadians(0))))))
    // .andThen(getCommandFor(4))
    // .andThen(new GrabberCommand(grabberSubsystem, true)));
    // command.setName("case 3");
    // return command;
    // }

}