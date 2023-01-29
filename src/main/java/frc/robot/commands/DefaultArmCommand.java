package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.utilities.Util.logf;

public class DefaultArmCommand extends CommandBase {
    ArmSubsystem armSubsystem;
    DoubleSupplier shoulderSupplier;
    DoubleSupplier extenderSupplier;
    boolean shoulderActive = false;
    boolean extenderActive = false;
    
    double SHOULDER_THRESHOLD = 500;
    double EXTENDER_THRESHOLD = 50;

    public enum State {
        IDLE, START_HOME_EXTENDER, HOMING_EXTENDER, HOMING_SHOULDER, READY
    }

    State homeState = State.START_HOME_EXTENDER;

    public DefaultArmCommand(ArmSubsystem armSubsystem,
            DoubleSupplier shoulderSupplier,
            DoubleSupplier extenderSupplier) {
                this.armSubsystem = armSubsystem;
                this.shoulderSupplier = shoulderSupplier;
                this.extenderSupplier = extenderSupplier;
        addRequirements(armSubsystem);
        
    }

    @Override
    public void initialize() {
        armSubsystem.lastExtenderStopPosition = armSubsystem.getExtenderPos();
        armSubsystem.lastShoulderStopPosition = armSubsystem.getShoulderPos();
    }

    @Override
    public void execute() {
        // if (Robot.count % 15 == 5) {
        //     double current = armSubsystem.shoulderMotor.getStatorCurrent();
        //     SmartDashboard.putNumber("ShlCur", current);
        //     boolean forwardLimit = armSubsystem.getForwardLimitSwitch(armSubsystem.shoulderMotor);
        //     SmartDashboard.putBoolean("ShlForL", forwardLimit);
        //     boolean reverseLimit = armSubsystem.getReverseLimitSwitch(armSubsystem.shoulderMotor);
        //     SmartDashboard.putBoolean("ShlRevL", reverseLimit);
        //     double position = armSubsystem.getShoulderPos();
        //     SmartDashboard.putNumber("ShlPos", position);
        // }

        
        // // Display Eextender Data
        // if (Robot.count % 15 == 10) {
        //     double current = armSubsystem.extenderMotor.getStatorCurrent();
        //     SmartDashboard.putNumber("ExtCur", current);
        //     boolean forwardLimit = armSubsystem.getForwardLimitSwitch(armSubsystem.extenderMotor);
        //     SmartDashboard.putBoolean("ExtForL", forwardLimit);
        //     boolean reverseLimit = armSubsystem.getReverseLimitSwitch(armSubsystem.extenderMotor);
        //     SmartDashboard.putBoolean("ExtRevL", reverseLimit);
        //     double position = armSubsystem.getExtenderPos();
        //     SmartDashboard.putNumber("ExtPos", position);
        // }


        if (homeState == State.START_HOME_EXTENDER) {
            armSubsystem.setExtenderSpeed(0.3);
            homeState = State.HOMING_EXTENDER;
        }
        if (homeState == State.HOMING_EXTENDER) {
            if (armSubsystem.getForwardLimitSwitch(armSubsystem.extenderMotor)) {
                homeState = State.HOMING_SHOULDER;
                armSubsystem.zeroEncoder(armSubsystem.extenderMotor);
                armSubsystem.setExtenderSpeed(0);  // KAG added 1/24
                logf("++++++++++++++++++++Extender Homed\n");
                armSubsystem.setExtenderSpeed(0);
                armSubsystem.setMotorToPosition(armSubsystem.extenderMotor, 0);
                armSubsystem.setShoulderSpeed(-0.3);
                logf("++++++++++++++++++++Start homing shoulder\n");
            }
        }
         

        if (armSubsystem.getForwardLimitSwitch(armSubsystem.extenderMotor)) {
            armSubsystem.zeroEncoder(armSubsystem.extenderMotor);
        }

        if (homeState == State.HOMING_SHOULDER) {
            if (armSubsystem.getReverseLimitSwitch(armSubsystem.shoulderMotor)) {
                homeState = State.READY;
                armSubsystem.zeroEncoder(armSubsystem.shoulderMotor);
                armSubsystem.setMotorToPosition(armSubsystem.shoulderMotor, 0);
                logf("++++++++++++++++++++Shoulder Homed\n");
                armSubsystem.setShoulderSpeed(0);
            }
            
        } 
        

        if (armSubsystem.getReverseLimitSwitch(armSubsystem.shoulderMotor)) {
            armSubsystem.zeroEncoder(armSubsystem.shoulderMotor);
        }

        if (homeState != State.READY) {
            return;
        }



        double shoulder = shoulderSupplier.getAsDouble();
        if (Math.abs(shoulder) > .05) {
            armSubsystem.setShoulderSpeed(shoulder * .3);
            shoulderActive = true;
            armSubsystem.lastShoulderStopPosition = armSubsystem.getShoulderPos();
            // logf("Shoulder Speed %.2f\n", left * .3);
        } else { 
            if (Math.abs(armSubsystem.getShoulderPos() - armSubsystem.lastShoulderStopPosition) > SHOULDER_THRESHOLD) {
                shoulderActive = false;
                armSubsystem.setShoulderSpeed(0);
                armSubsystem.setEncoderPosition(armSubsystem.shoulderMotor,armSubsystem.lastShoulderStopPosition);
            }
        }

        double extender = extenderSupplier.getAsDouble();
        SmartDashboard.putNumber("Extender Speed", extender);
        if (Math.abs(extender) > .05) {
            extenderActive = true;
            armSubsystem.setExtenderSpeed(extender * .9);
            armSubsystem.lastExtenderStopPosition = armSubsystem.getExtenderPos();
            if (Robot.count % 20 == 0) {
              logf("Extender Speed %.2f\n"  ,extender * .9);
            }
        } else {
            if (Math.abs(armSubsystem.getExtenderPos() - armSubsystem.lastExtenderStopPosition) > EXTENDER_THRESHOLD) {
                extenderActive = false;
                armSubsystem.setExtenderSpeed(0);
                //logf("*****-----******* current pos = %.2f setting the extender pos to %.2f\n", armSubsystem.getExtenderPos(), armSubsystem.lastExtenderStopPosition);
                // TODO this is not correct -- test with a call below
                // armSubsystem.setMotorToPosition(armSubsystem.extenderMotor, armSubsystem.lastExtenderStopPosition);
                //armSubsystem.setMotorToPosition(armSubsystem.extenderMotor,-500 );
            }
        }
    }
}
