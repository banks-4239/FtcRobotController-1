package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(group = "advanced")
@Config
@Disabled
public class Common extends LinearOpMode {

    //change values here to change everywhere
    int armPositionHighScore = 3864;
    int armPositionMidScore = 2748;
    int armPositionLowScore = 1977;
    int armPositionConeStack = 593;
    int armPositionStartingLocation = 0;
    int armMotorVelocity = 1500;
    boolean clawOpen = false;
    boolean yPressed = false;
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void drive(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
                robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
        robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
        robot.rightRear.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
        robot.leftRear.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
    }
    public void initialization(){

        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();
    }
    public void calibrateArm(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        while(robot.armHeightSwitch.getState() && opModeIsActive() && !isStopRequested()){
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(-200);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setVelocity(armMotorVelocity);
        }while(!robot.armHeightSwitch.getState() && opModeIsActive() && !isStopRequested()){
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setVelocity(armMotorVelocity);
        }
        robot.moveArmTo(0);
        }
    public void openClaw() {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.clawServo.setPosition(1);
    }
    public void closeClaw() {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.clawServo.setPosition(0.9);
    }
    public void goUpABit(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setTargetPosition(robot.armMotor.getCurrentPosition()-50);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setVelocity(armMotorVelocity);
        robot.closeClaw();
        clawOpen = false;
    }
    public void goDownABit(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setTargetPosition(robot.armMotor.getCurrentPosition()+50);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setVelocity(armMotorVelocity);
        robot.closeClaw();
        clawOpen = false;
    }
    public void moveArmTo(int armPosition){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.armMotor.setTargetPosition(armPosition);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(armMotorPower);
        //ticks/second
        robot.armMotor.setVelocity(armMotorVelocity);
        closeClaw();
    }
    public void clawOperation(boolean isPressed){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        if(isPressed && clawOpen && !yPressed){
            robot.closeClaw();
            clawOpen = false;
            yPressed = true;
        }
        if (isPressed && !clawOpen && !yPressed) {
            robot.openClaw();
            clawOpen = true;
            yPressed = true;
        }
        if (isPressed && yPressed) {
            yPressed = false;
        }
    }
    public void driveRobot(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
        robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
        robot.rightRear.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
        robot.leftRear.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
//        robot.setWeightedDrivePower(
//                new Pose2d(
//                        //-forward/backward
//                        -(gamepad1.left_stick_y/2),
//                        //-strafe
//                        -(gamepad1.left_stick_x/2),
//                        //-rotate
//                        -(gamepad1.right_stick_x/2)
//                )
//        );
    }
    public void resetArmEncoder(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void getTelemetry(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        // Update everything. Odometry. Etc.
        robot.update();

        // Read pose
        //Pose2d poseEstimate = robot.getPoseEstimate();
        // Print pose to telemetry
        //telemetry.addData("x", poseEstimate.getX());
        //telemetry.addData("y", poseEstimate.getY());
        //telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("At Bottom: ", !robot.armHeightSwitch.getState());
        telemetry.addData("Claw is open = ", clawOpen);
        telemetry.addData("Arm Height", robot.armMotor.getCurrentPosition());
        telemetry.update();
    }
}