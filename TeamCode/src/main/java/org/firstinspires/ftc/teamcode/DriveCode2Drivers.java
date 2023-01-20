package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "advanced")
public class DriveCode2Drivers extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        robot.setPoseEstimate(PoseStorage.currentPose);

        //change values here to change everywherearmHeightSwitch;
        int armTarget = 0;
        int downALittle = 0;
        int armPositionHighScore = 3214;
        int armPositionMidScore = 2397;
        int armPositionLowScore = 1577;
        int armPositionConeStack = 1008;
        int armPositionConeStackDifference = -125;
        int[] armPositionConeStacks = new int[5];
        int armPositionStartingLocation = 10;//Used to be zero
        double armMotorPower = 0.5;
        armPositionConeStacks[0] = armPositionConeStack+0*armPositionConeStackDifference;
        armPositionConeStacks[1] = armPositionConeStack+1*armPositionConeStackDifference;
        armPositionConeStacks[2] = armPositionConeStack+2*armPositionConeStackDifference;
        armPositionConeStacks[3] = armPositionConeStack+3*armPositionConeStackDifference;
        armPositionConeStacks[4] = armPositionConeStack+4*armPositionConeStackDifference;
        boolean clawOpen = false;
        boolean yPressed = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.left_bumper){
                downALittle = 600;
            }
            else{
                downALittle = 0;
            }
            //////////////////////////////
           if(gamepad2.dpad_up){
                armTarget = armPositionHighScore;
                //robot.moveArmTo(armPositionHighScore);
            }else if(gamepad2.dpad_down){
                armTarget = armPositionStartingLocation;
                //robot.moveArmTo(armPositionStartingLocation);
            }else if(gamepad2.dpad_left){
                armTarget = armPositionMidScore;
                //robot.moveArmTo(armPositionMidScore);
            }else if(gamepad2.dpad_right){
                armTarget = armPositionLowScore;
                //robot.moveArmTo(armPositionLowScore);
            }
            else if(gamepad2.start){
                calibrateArm();
            }
            else if(gamepad2.y){
                robot.moveArmTo(armPositionConeStacks[0]);
                clawOpen = false;
            }else if(gamepad2.x){
                robot.moveArmTo(armPositionConeStacks[1]);
                clawOpen = false;
            }else if(gamepad2.a){
                robot.moveArmTo(armPositionConeStacks[2]);
                clawOpen = false;
            }else if(gamepad2.b){
                robot.moveArmTo(armPositionConeStacks[3]);
                clawOpen = false;
            }
            robot.moveArmTo(armTarget-downALittle);

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
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Claw is open = ", clawOpen);
            telemetry.addData("Arm Height", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
    }    public void calibrateArm(){
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


}