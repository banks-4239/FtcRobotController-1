package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(group = "advanced")
public class ArmCalibration2 extends Common {
    ;
    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        //SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive();
//            SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
//            robot.setWeightedDrivePower(
//                    new Pose2d(-(gamepad1.left_stick_y/2),-(gamepad1.left_stick_x/2),-(gamepad1.right_stick_x/2))
//            );
            if(gamepad1.b){
                calibrateArm();
            }
            if(gamepad2.dpad_up){
                goUpABit();
            }else if(gamepad2.dpad_down){
                goDownABit();
            }else if(gamepad1.dpad_up){
                moveArmTo(armPositionHighScore);
            }else if(gamepad1.dpad_down){
                moveArmTo(armPositionStartingLocation);
            }else if(gamepad1.dpad_left){
                moveArmTo(armPositionMidScore);
            }else if(gamepad1.dpad_right){
                moveArmTo(armPositionLowScore);
            }
            if(gamepad1.x){resetArmEncoder();}
            clawOperation(gamepad1.y);
            getTelemetry();
        }

    }

//        robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/2);s
//        robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
//        robot.rightRear.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
//        robot.leftRear.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/adfsdfdas

}