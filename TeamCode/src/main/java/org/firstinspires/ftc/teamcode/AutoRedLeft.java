package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

public class AutoRedLeft extends LinearOpMode {
    double speed = 0.5;
    int armPositionHighScore = -2867;
    int armPositionMidScore = -2239;
    int armPositionLowScore = -1593;
    int armPositionStartingLocation = 0;
    int armPositionConeStack = -850;
    double armMotorPower = 0.5;
    int armPositionLiftConeStack = -550;
    int armPositionConeStackDifference = 125;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Servo clawServo = hardwareMap.get(Servo.class, "claw");
        DcMotor armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(-33, -63, Math.toRadians(90));
        Vector2d redLeft1 = new Vector2d(-11.6, -63);
        Vector2d redLeft2 = new Vector2d(-11.6, -36);
        //rotate-90
        Vector2d redLeftScoreSetup = new Vector2d(-11.6, -24.8);
        Vector2d redLeftScore = new Vector2d(-7.4, -24.8);
        //redleftscoresetup
        Vector2d redLeft3 = new Vector2d(-11.6, -12.2);
        //rotate180
        Vector2d redLeftConeStackSetup = new Vector2d(-56, -12.2);
        Vector2d redLeftConeStack = new Vector2d(-62, -12.2);
        //redleftconestacksetup
        //redLeft3
        //redLeftScoreSetup

        robot.setPoseEstimate(startPose);

//
//                .lineTo(new Vector2d(-11.6, -12.2), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .turn(Math.toRadians(180))
//                .lineTo(new Vector2d(-63, -12.2), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> {
//                   robot.closeClaw();
//                    robot.raiseArmToLow();
//                })
//
//                .lineTo(new Vector2d(-11.6, -12.2), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .turn(Math.toRadians(180))
//                .lineTo(new Vector2d(-11.6, -23.8), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> {
//                    robot.raiseArmToHigh();
//                })
//
//                .lineTo(new Vector2d(-7.4, -23.8), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> {
//                    robot.openClaw();
//
//                    sleep(500);
//                    robot.closeClaw();
//                  //  robot.moveArmTo(armPositionConeStack+50);
//                })
//                .lineTo(new Vector2d(-11.6, -23.8), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//
//                .addDisplacementMarker(() -> {
//                    robot.moveArmTo(armPositionStartingLocation);
//                    robot.raiseArmToStart();
//                })
//                .waitSeconds(10)

//                .addDisplacementMarker(() -> {
//                    robot.
//                    clawServo.setPosition(0.9);
//                })
//
//                .lineTo(new Vector2d(-11.6, -12.2), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .turn(Math.toRadians(180))
//                .lineTo(new Vector2d(-63, -12.2), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> {
//                    clawServo.setPosition(1);
//                    //liftarmpastconestack
//                })
//
//                .lineTo(new Vector2d(-11.6, -12.2), SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 2), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .turn(Math.toRadians(180))
        //              .build();
        TrajectorySequence redLeftStartToHighPole = robot.trajectorySequenceBuilder(startPose)
                //strafe to right
                .lineTo(redLeft1, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL  ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addDisplacementMarker(() -> {
                    robot.moveArmTo(armPositionHighScore);
                })
                //move forward by 1 square
                .lineTo(redLeft2, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //turns to the right
                .turn(Math.toRadians(-90))
                //strafe left to pole (high)
                .lineTo(redLeftScoreSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory redLeftMoveToScore = robot.trajectoryBuilder(redLeftStartToHighPole.end())
                .lineTo(redLeftScore, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory redLeftBackOffOfPole = robot.trajectoryBuilder(redLeftMoveToScore.end())
                .lineTo(redLeftScoreSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence redLeftGoToConeStack = robot.trajectorySequenceBuilder(redLeftBackOffOfPole.end())
                .lineTo(redLeft3, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(180))
                .lineTo(redLeftConeStackSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.openClaw();
                })
                .lineTo(redLeftConeStack, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 5), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
            Trajectory redLeftBackOffOfConeStack = robot.trajectoryBuilder(redLeftGoToConeStack.end())
                    .lineTo(redLeftConeStackSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL / 5), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            TrajectorySequence redLeftConeStackSetupToHighPoleSetup = robot.trajectorySequenceBuilder(redLeftBackOffOfConeStack.end())
                    .lineTo(redLeft3, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .turn(Math.toRadians(180))
                    .lineTo(redLeftScoreSetup, SampleMecanumDrive.getVelocityConstraint((DriveConstants.MAX_VEL ), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

        //    TrajectorySequence name = robot.trajectorySequenceBuilder(redLeftStartToHighPole.end())
        //            .build();

        //    Trajectory name = robot.trajectoryBuilder(redLeftStartToHighPole.end())
        //            .build();


        robot.closeClaw();
        waitForStart();


        robot.followTrajectorySequence(redLeftStartToHighPole);

        for(int i = 0;i<5;i++) {
//            robot.moveArmTo(armPositionHighScore);
//            sleep(2500);


            robot.followTrajectory(redLeftMoveToScore);
            sleep(250);
            robot.openClaw();
            sleep(250);
            robot.followTrajectory(redLeftBackOffOfPole);
            robot.moveArmTo(armPositionConeStack+armPositionConeStackDifference*i);

            robot.followTrajectorySequence(redLeftGoToConeStack);
            robot.closeClaw();
            sleep(250);
            //liftConeFromStack
            robot.moveArmTo(armPositionConeStack+armPositionConeStackDifference*i + armPositionLiftConeStack);
            sleep(1000);

            robot.followTrajectory(redLeftBackOffOfConeStack);
            robot.moveArmTo(armPositionHighScore);
            robot.followTrajectorySequence(redLeftConeStackSetupToHighPoleSetup);
        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = robot.getPoseEstimate();
    }

}