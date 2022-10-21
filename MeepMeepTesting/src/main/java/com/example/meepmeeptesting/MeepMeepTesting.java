
package com.example.meepmeeptesting;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.noahbres.meepmeep.MeepMeep;
        import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
        import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

        import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Vector2d blueSignalRight = new Vector2d(-35.4,35.4);
        Pose2d startLocation = new Pose2d(-33,-63,Math.toRadians(90));
        Pose2d startLocationRedRight = new Pose2d(33,-63,Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startLocationRedRight)
                                .lineTo(new Vector2d(11.8,-63))
                                .lineTo(new Vector2d(11.6,-36))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(11.6,-23.8))
                                //liftarm
                                .lineTo(new Vector2d(7.4,-23.8))
                                //openclaw
                                //closeclaw
                                .lineTo(new Vector2d(11.6,-23.8))
                                //lowerarmtocorrectconestackheight
                                //openclaw
                                .lineTo(new Vector2d(11.6,-12.2))
                                .turn(Math.toRadians(180))
                                .lineTo(new Vector2d(63,-12.2))
                                //closeclaw
                                //liftarmpastconestack
                                .lineTo(new Vector2d(11.6,-12.2))
                                .turn(Math.toRadians(180))
                                .lineTo(new Vector2d(11.6,-23.8))
                                //liftarm
                                .lineTo(new Vector2d(7.4,-23.8))
                                //openclaw
                                //closeclaw
                                .lineTo(new Vector2d(11.6,-23.8))
                                //lowerarmtocorrectconestackheight
                                //openclaw
                                .lineTo(new Vector2d(11.6,-12.2))
                                .turn(Math.toRadians(180))
                                .lineTo(new Vector2d(63,-12.2))
                                //closeclaw
                                //liftarmpastconestack
                                .lineTo(new Vector2d(11.6,-12.2))
                                .turn(Math.toRadians(180))




                        //.forward(40)
                        //.splineTo(blueSignalRight,Math.toRadians(180))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}