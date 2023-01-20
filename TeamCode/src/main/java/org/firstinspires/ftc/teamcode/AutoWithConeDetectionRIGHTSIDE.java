/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous
public class AutoWithConeDetectionRIGHTSIDE extends LinearOpMode {
    //RobotReference robotStuff = new RobotReference();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;

    int park_1 = 8; // Tag ID 18 from the 36h11 family
    int park_2 = 13;
    int park_3 = 21;
    int xReflect = 1;
    int rotateReflect;
    public final int armPositionHighScore = 3214;
    public final int armPositionMidScore = 2397;
    public final int armPositionLowScore = 1577;
    public final int armPositionStartingLocation = 0;
    public final int armPositionConeStack = 650;
    public final int armPositionConeStackDifference = -150;
    double armMotorPower = 0.5;
    int armPositionLiftConeStack = 593;
    double clawOffset = 1.5;
    double tileWidth = 23.5;
    double speedConstant = 0.5;
    double slow = 0.5;
    int[] armPositionConeStacks = new int[5];
    int conesToScore = 3;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        for (int i = 0; i < 5; i++) {
            armPositionConeStacks[i] = armPositionConeStack + i * armPositionConeStackDifference;
        }
        //robotStuff.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);

        robot.getConstants();
        Pose2d StartPose = new Pose2d(36.25 * xReflect, -62, Math.toRadians(90));
        Vector2d zone1 = new Vector2d(35.25 * xReflect - 24, -11.5);
        Vector2d zone2 = new Vector2d(35.25 * xReflect, -7.5);
        Vector2d zone3 = new Vector2d(35.25 * xReflect + 24, -6.5);
//-30.42253007 for the x, and -2.781101 for the y values of the score.
        Pose2d Score = new Pose2d(3.5 * xReflect, -3, Math.toRadians(135));
        //Pose2d Score1 = new Pose2d(-30.42, -2.78, Math.toRadians(45));
        //Pose2d RedLeftConeStack = new Pose2d(-64 * xReflect, -5.75, Math.toRadians(rotateReflect - 180));
        drive.setPoseEstimate(StartPose);

//        TrajectorySequence startToZone2 = drive.trajectorySequenceBuilder(StartPose)
//                .lineTo(zone2)
//                .turn(Math.toRadians(-45))
//                .build();
//        TrajectorySequence zone2ToScore = drive.trajectorySequenceBuilder(startToZone2.end())
//                .lineToLinearHeading(Score, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
//                .build();
        TrajectorySequence startToScore = drive.trajectorySequenceBuilder(StartPose)
                .splineTo(zone2, Math.toRadians(90))
                .splineToSplineHeading(Score, Math.toRadians(135), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
//                .splineTo(new Vector2d(-30, -4), Math.toRadians(45),SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL * speedConstant), DriveConstants.MAX_ANG_VEL * speedConstant, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * speedConstant))
                .build();
//        TrajectorySequence scoreToZone2 = drive.trajectorySequenceBuilder(zone2ToScore.end())
//                .lineToLinearHeading(Zone2)
//                .build();

//        TrajectorySequence scoreToConeStack = drive.trajectorySequenceBuilder(startToScore.end())
//                .lineTo(zone2)
//                .addDisplacementMarker(5, () -> {
//                    drive.closeClaw();
//                })
//                .turn(Math.toRadians(135))
//                .splineToSplineHeading(new Pose2d(zone1, Math.toRadians(180)), Math.toRadians(180))
//                .addDisplacementMarker(20, () -> {
//                    drive.openClaw();
//                })
//                .forward(2)
//                .build();

//        TrajectorySequence Zone1ToScore = drive.trajectorySequenceBuilder(scoreToConeStack.end())
//                .lineTo(zone2)
////                .turn(Math.toRadians(-60))
//                .lineToSplineHeading(Score)
//
//                .build();


//        TrajectorySequence scoreToZone2 = drive.trajectorySequenceBuilder(Zone1ToScore.end())
//                .lineTo(zone2)
//                .turn(Math.toRadians(45))
//                .build();
//        TrajectorySequence zone2ToZone1 = drive.trajectorySequenceBuilder(scoreToZone2.end())
//                .lineTo(zone1)
//                .build();
//        TrajectorySequence zone2ToZone3 = drive.trajectorySequenceBuilder(scoreToZone2.end())
//                .lineTo(zone3)
//                .build();

//        TrajectorySequence startToZone1 = drive.trajectorySequenceBuilder(startPose)
//                .lineTo(zone2)
//                .lineTo(zone1)
//                .build();



        /*
         * The INIT-loop:
         * .addDisplacementMarker(() -> {
                    robot.moveArmTo(armPositionHighScore);
                })
         * This REPLACES waitForStart!
         */
        drive.moveArmTo(armPositionStartingLocation);//This line used to be nonexistent
        sleep(250);//This line used to be nonexistent
        drive.closeClaw();

        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == park_1 || tag.id == park_2 || tag.id == park_3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */


        drive.moveArmTo(armPositionHighScore);
        //drive.followTrajectorySequence(startToZone2);
        //drive.followTrajectorySequence(zone2ToScore);
        drive.followTrajectorySequence(startToScore);

        for (int i = 0; i < conesToScore - 1 && opModeIsActive() && !isStopRequested(); i++) {
            drive.moveArmTo(armPositionConeStacks[i]);
            sleep(500);
            drive.openClaw();
            sleep(250);
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
            TrajectorySequence scoreToConeStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(zone2)
                    .turn(Math.toRadians(-135))
                    .lineToSplineHeading(new Pose2d(zone3, Math.toRadians(360)))
                    .forward(2)
                    .build();
            drive.followTrajectorySequence(scoreToConeStack);
            drive.closeClaw();
            sleep(100);
            drive.moveArmTo(armPositionHighScore);
            sleep(250);
            Pose2d poseEstimate2 = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate2.getX());
            telemetry.addData("y", poseEstimate2.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate2.getHeading()));
            telemetry.update();
            TrajectorySequence Zone1ToScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(zone2)
                    .turn(Math.toRadians(135))
                    .lineToSplineHeading(Score)

                    .build();
            drive.followTrajectorySequence(Zone1ToScore);
        }
        sleep(500);
        drive.moveArmTo(armPositionStartingLocation);
        sleep(500);
        drive.openClaw();
        sleep(250);
        TrajectorySequence scoreToZone2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(zone2)
                .turn(Math.toRadians(-45))
                .build();
        drive.followTrajectorySequence(scoreToZone2);
        if (tagOfInterest == null) {

        } else if (tagOfInterest.id == park_1) {
            TrajectorySequence zone2ToZone1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(zone1)
                    .build();
            drive.followTrajectorySequence(zone2ToZone1);
        } else if (tagOfInterest.id == park_3) {
            TrajectorySequence zone2ToZone3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(zone3)
                    .build();
            drive.followTrajectorySequence(zone2ToZone3);
        }



        /*
        if(tagOfInterest == null)
        {
            //code to run when nothing was ever detected
            // drive.followTrajectory(startToZone2);
        }
        else {

            if(tagOfInterest.id == park_1)
            {
                telemetry.clearAll();
                telemetry.addData("zone","1");
                telemetry.update();
                drive.followTrajectorySequence(startToZone1);
            }
            else if(tagOfInterest.id == park_2)
            {
                telemetry.clearAll();
                telemetry.addData("zone","2");
                telemetry.update();

                drive.followTrajectory(startToZone2);
            }
            else if(tagOfInterest.id == park_3)
            {
                telemetry.clearAll();
                telemetry.addData("zone","3");
                telemetry.update();
                drive.followTrajectorySequence(startToZone3);
            }
        }
        */

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    void waitUntilAisPressed() {
        while (!gamepad1.a && !isStopRequested() && opModeIsActive()) {
        }

    }
public void DisplayPose(){

}
    public void calibrateArm() {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        while (robot.armHeightSwitch.getState() && opModeIsActive() && !isStopRequested()) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(200);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setVelocity(1500);
        }
        while (!robot.armHeightSwitch.getState() && opModeIsActive() && !isStopRequested()) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(-100);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setVelocity(1500);
        }
        robot.moveArmTo(0);
    }
}