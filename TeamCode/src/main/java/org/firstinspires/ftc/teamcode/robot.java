package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Example opmode demonstrating how to hand-off the pose from your autonomous opmode to your teleop
 * by passing the data through a static class.
 * <p>
 * This is required if you wish to read the pose from odometry in teleop and you run an autonomous
 * sequence prior. Without passing the data between each other, teleop isn't completely sure where
 * it starts.
 * <p>
 * This example runs the same paths used in the SplineTest tuning opmode. After the trajectory
 * following concludes, it simply sets the static value, `PoseStorage.currentPose`, to the latest
 * localizer reading.
 * However, this method is not foolproof. The most immediate problem is that the pose will not be
 * written to the static field if the opmode is stopped prematurely. To work around this issue, you
 * need to continually write the pose to the static field in an async trajectory follower. A simple
 * example of async trajectory following can be found at
 * https://www.learnroadrunner.com/advanced.html#async-following
 * A more advanced example of async following can be found in the AsyncFollowingFSM.java class.
 * <p>
 * The other edge-case issue you may want to cover is saving the pose value to disk by writing it
 * to a file in the event of an app crash. This way, the pose can be retrieved and set even if
 * something disastrous occurs. Such a sample has not been included.
 */
@Autonomous
@Disabled
public class robot extends LinearOpMode {

    int xReflect;
    int rotateReflect;
    int armPositionHighScore = -2669;
    int armPositionMidScore = -2086;
    int armPositionLowScore = -1415;
    int armPositionStartingLocation = 0;
    int armPositionConeStack = -635;
    double armMotorPower = 0.5;
    int armPositionLiftConeStack = -550;
    int armPositionConeStackDifference = 125;
    double clawOffset = 1.5;
    double tileWidth = 23.5;
    double speedConstant = 0.5;
    double slow = 0.5;
    Servo clawServo = hardwareMap.get(Servo.class, "claw");
    DcMotor armMotor = hardwareMap.get(DcMotorEx.class, "arm");
    DigitalChannel armHeightSwitch = hardwareMap.get(DigitalChannel .class, "magswitch");
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {


    }
    public void calibrateArm(){
        while(drive.armHeightSwitch.getState()){
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition(100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.1);
        }while(!drive.armHeightSwitch.getState()){
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition(-100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.25);
        }
        moveArmTo(0);
    }
    public static void getConstants(){
        int xReflect;
        int rotateReflect;
        int armPositionHighScore = -2669;
        int armPositionMidScore = -2086;
        int armPositionLowScore = -1415;
        int armPositionStartingLocation = 0;
        int armPositionConeStack = -635;
        double armMotorPower = 0.5;
        int armPositionLiftConeStack = -550;
        int armPositionConeStackDifference = 125;
        double clawOffset = 1.5;
        double tileWidth = 23.5;
        double speedConstant = 0.5;
        double slow = 0.5;
    }
    public void moveArmTo(int armPosition){
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(armMotorPower);
        closeClaw();
    }
    public void closeClaw() {
        clawServo.setPosition(1);
    }
    public void openClaw() {
        clawServo.setPosition(0.9);
    }
}