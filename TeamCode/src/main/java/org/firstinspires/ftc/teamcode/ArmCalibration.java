package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "advanced")
public class ArmCalibration extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve our poserobot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); from the PoseStorage.currentPose static field
        // this is what we get from autonomous

        robot.setPoseEstimate(new Pose2d(36.25, -62, Math.toRadians(90)));
        //robot.setPoseEstimate(PoseStorage.currentPose);

         //change values here to change everywhere
        int armTarget = 0;
        int downALittle = 0;
        int armPositionHighScore = 3114;
        int armPositionMidScore = 2297;
        int armPositionLowScore = 1477;
        int armPositionStartingLocation = 0;
        double armMotorPower = 0.5;

        boolean clawOpen = false;
        boolean yPressed = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.b){
                calibrateArm();
            }
            /////////////////////////

            if(gamepad1.left_bumper){
                downALittle = 600;
            }
            else{
                downALittle = 0;
            }
            //////////////////////////////
            if(gamepad2.dpad_up){
                armTarget = robot.armMotor.getCurrentPosition()-50;
                //robot.moveArmTo(robot.armMotor.getCurrentPosition()-50);
            }else if(gamepad2.dpad_down){
                armTarget = robot.armMotor.getCurrentPosition()+50;
                //robot.moveArmTo(robot.armMotor.getCurrentPosition()+50);
            }
            else if(gamepad1.dpad_up){
                armTarget = armPositionHighScore;
                //robot.moveArmTo(armPositionHighScore);
            }else if(gamepad1.dpad_down){
                armTarget = armPositionStartingLocation;
                //robot.moveArmTo(armPositionStartingLocation);
            }else if(gamepad1.dpad_left){
                armTarget = armPositionMidScore;
                //robot.moveArmTo(armPositionMidScore);
            }else if(gamepad1.dpad_right){
                armTarget = armPositionLowScore;
                //robot.moveArmTo(armPositionLowScore);
        }
            robot.moveArmTo(armTarget-downALittle);
//////////////////////////////////////////
            if(gamepad1.x){
                robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.y && clawOpen && !yPressed){
                robot.closeClaw();
                clawOpen = false;
                yPressed = true;
            }
            if (gamepad1.y && !clawOpen && !yPressed) {
                robot.openClaw();
                clawOpen = true;
                yPressed = true;
            }
            if (!gamepad1.y && yPressed) {
                yPressed = false;
            }

            robot.setWeightedDrivePower(
                    new Pose2d(
                            //-forward/backward
                            -(gamepad1.left_stick_y/2),
                            //-strafe
                            -(gamepad1.left_stick_x/2),
                            //-rotate
                            -(gamepad1.right_stick_x/2)
                    )
            );

            // Update everything. Odometry. Etc.
            robot.update();

            // Read pose
            Pose2d poseEstimate = robot.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("At Bottom: ", !robot.armHeightSwitch.getState());
//            telemetry.addData("Claw is open = ", clawOpen);
            telemetry.addData("Arm Height", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
    public void calibrateArm(){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        while(robot.armHeightSwitch.getState() && opModeIsActive() && !isStopRequested()){
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(-200);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(0.25);
        }while(!robot.armHeightSwitch.getState() && opModeIsActive() && !isStopRequested()){
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(0.25);
        }
        robot.moveArmTo(0);
        }
public void bringArmDownSlightly(){


}
}