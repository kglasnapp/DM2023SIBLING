package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;



public class DefaultArmCommand extends CommandBase {
    ArmSubsystem armSubsystem;
    DoubleSupplier shoulderSupplier;
    DoubleSupplier extenderSupplier;
    DoubleSupplier shoulderSupplier2;
    DoubleSupplier extenderSupplier2;
    boolean shoulderActive = false;
    boolean extenderActive = false;

    double SHOULDER_THRESHOLD = 35000;
    double EXTENDER_THRESHOLD = 50;

    public enum State {
        IDLE, START_HOME_EXTENDER, HOMING_EXTENDER, HOMING_EXTENDER_2, HOMING_SHOULDER, READY
    }

    State homeState = State.START_HOME_EXTENDER;

    public DefaultArmCommand(ArmSubsystem armSubsystem,
            DoubleSupplier shoulderSupplier,
            DoubleSupplier extenderSupplier,
            DoubleSupplier shoulderSupplier2,
            DoubleSupplier extenderSupplier2) {
        this.armSubsystem = armSubsystem;
        this.shoulderSupplier = shoulderSupplier;
        this.extenderSupplier = extenderSupplier;
        this.shoulderSupplier2 = shoulderSupplier2;
        this.extenderSupplier2 = extenderSupplier2;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {
        armSubsystem.lastExtenderStopPosition = armSubsystem.getExtenderPos();
        armSubsystem.lastShoulderStopPosition = armSubsystem.getShoulderPos();
    }

    @Override
    public void execute() {
        if (Robot.count % 30 == 5) {
            SmartDashboard.putString("ArmState", homeState.toString());
        }
        if (homeState == State.START_HOME_EXTENDER) {
            armSubsystem.setExtenderSpeed(-0.6);
            homeState = State.HOMING_EXTENDER;
        }
        if (homeState == State.HOMING_EXTENDER) {
            if (armSubsystem.getReverseLimitSwitch(armSubsystem.extenderMotor)) {
                homeState = State.HOMING_SHOULDER;
                armSubsystem.zeroEncoder(armSubsystem.extenderMotor);
                armSubsystem.setExtenderSpeed(0); // KAG added 1/24
               // logf("++++++++++++++++++++Extender Homed\n");            
                armSubsystem.setMotorToPosition(armSubsystem.extenderMotor, 0);
                armSubsystem.setShoulderSpeed(-0.3);
            }
        }
        if (homeState == State.HOMING_EXTENDER_2) {
            if (!armSubsystem.getReverseLimitSwitch(armSubsystem.extenderMotor)) {                
                homeState = State.HOMING_SHOULDER;
                armSubsystem.zeroEncoder(armSubsystem.extenderMotor);
                armSubsystem.setExtenderSpeed(0); // KAG added 1/24
               // logf("++++++++++++++++++++Extender Homed\n");
                armSubsystem.setExtenderSpeed(-0.1);
                armSubsystem.setMotorToPosition(armSubsystem.extenderMotor, 0);
                armSubsystem.setShoulderSpeed(-0.3);
               // logf("++++++++++++++++++++Start homing shoulder\n");
            }
        }

       
        

        if (homeState == State.HOMING_SHOULDER) {
            if (armSubsystem.getReverseLimitSwitch(armSubsystem.shoulderMotor)) {
                homeState = State.READY;
                armSubsystem.zeroEncoder(armSubsystem.shoulderMotor);
                armSubsystem.setMotorToPosition(armSubsystem.shoulderMotor, 0);
                //logf("++++++++++++++++++++Shoulder Homed\n");
                armSubsystem.setShoulderSpeed(0);
            }
        }

        if (armSubsystem.getReverseLimitSwitch(armSubsystem.shoulderMotor)) {
            armSubsystem.zeroEncoder(armSubsystem.shoulderMotor);
        }

        // if (armSubsystem.getReverseLimitSwitch(armSubsystem.extenderMotor)) {
        //     armSubsystem.zeroEncoder(armSubsystem.extenderMotor);
        // }

        if (homeState != State.READY) {
            return;
        }

        double shoulderButtonPressSpeed = shoulderSupplier.getAsDouble();
        if (shoulderButtonPressSpeed == 0) {
            shoulderButtonPressSpeed = shoulderSupplier2.getAsDouble();
        }
        if (Math.abs(shoulderButtonPressSpeed) > .05) {
            armSubsystem.setShoulderSpeed(shoulderButtonPressSpeed * 0.6);
            shoulderActive = true;
            armSubsystem.lastShoulderStopPosition = armSubsystem.getShoulderPos();
           // logf("Shoulder Speed %.2f\n", shoulder * .3);
        } else {
            armSubsystem.setShoulderSpeed(0);
            shoulderActive = false;
            // if (shoulderActive) {
            //     if (Math.abs(
            //             armSubsystem.getShoulderPos() - armSubsystem.lastShoulderStopPosition) > SHOULDER_THRESHOLD) {
            //         shoulderActive = false;
                    
            //         armSubsystem.setEncoderPosition(armSubsystem.shoulderMotor, armSubsystem.lastShoulderStopPosition);
            //     }
            // }
        }

        double extenderButtonPressSpeed = extenderSupplier.getAsDouble();
        if (extenderButtonPressSpeed == 0) {
            extenderButtonPressSpeed = extenderSupplier2.getAsDouble();
        }
        // as we are not zeroing when we hit the limit switch anymore
        // we don't need to prevent it from going beyond zero.
        // // if we are close to zero on the encoder, and we try to go farther
        // // back, we stop.
        // if (armSubsystem.getExtenderPos() < 20000 && extender < 0) {
        //     extender = 0;
        // }
        // SmartDashboard.putNumber("Extender Speed", extender);
        if (Math.abs(extenderButtonPressSpeed) > .05 && armSubsystem.getShoulderPos() > ExtenderCommand.SHOULDER_THRESHOLD) {
            extenderActive = true;
            armSubsystem.setExtenderSpeed(extenderButtonPressSpeed * 1.0);
            armSubsystem.lastExtenderStopPosition = armSubsystem.getExtenderPos();
           // logf("Extender Speed %.2f\n", extender * .9);
        } else {
            armSubsystem.setExtenderSpeed(0);
            extenderActive = false;
            // Was taking too much time to set speed to zero so add extenderActive test
            // if (extenderActive) {
            //     if (Math.abs(
            //             armSubsystem.getExtenderPos() - armSubsystem.lastExtenderStopPosition) > EXTENDER_THRESHOLD) {
            //         extenderActive = false;
            //         armSubsystem.setExtenderSpeed(0);
            //         // armSubsystem.setMotorToPosition(armSubsystem.extenderMotor,
            //         // armSubsystem.lastExtenderStopPosition);
            //         // armSubsystem.setMotorToPosition(armSubsystem.extenderMotor,-500 );
            //     }
            // }
        }
    }
}
