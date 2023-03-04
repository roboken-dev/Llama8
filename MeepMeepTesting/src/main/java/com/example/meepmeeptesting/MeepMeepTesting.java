package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double TRACK_WIDTH = 13.23; // in
    public static double MAX_VEL = 52;
    public static double MAX_ACCEL = 52.48291908330528;
    public static double MAX_ANG_VEL = 3.5;//Math.toRadians(290.9352408290081);
    public static double MAX_ANG_ACCEL = Math.toRadians(240.5639808);
    public static void main(String[] args) {
        MecanumVelocityConstraint slowVelocity = new MecanumVelocityConstraint(15, TRACK_WIDTH);
        ProfileAccelerationConstraint slowAccel = new ProfileAccelerationConstraint(MAX_ACCEL);

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, 65, Math.toRadians(270)))
                                //drop pre loaded cone on high pole
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-10, 28), Math.toRadians(270))
                               // .splineToConstantHeading(new Vector2d(-26.75, 6.5), Math.toRadians(225))

                                //pick up first cone from pile
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-30.5, 10.5), Math.toRadians(135))
                                .splineToSplineHeading(new Pose2d(-35.5, 12, Math.toRadians(170)), Math.toRadians(170))
                                .splineToSplineHeading(new Pose2d(-64.5, 10, Math.toRadians(180)), Math.toRadians(180),
                                        slowVelocity, slowAccel)

                                //drop cone on high pole
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(-40, 13), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-26.75, 6.5, Math.toRadians(270)), Math.toRadians(270),
                                        slowVelocity, slowAccel)

                                //pick up second cone from pile
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-30.5, 10.5), Math.toRadians(135))
                                .splineToSplineHeading(new Pose2d(-35.5, 12, Math.toRadians(170)), Math.toRadians(170))
                                .splineToSplineHeading(new Pose2d(-64.5, 10, Math.toRadians(180)), Math.toRadians(180),
                                        slowVelocity, slowAccel)

                                //drop cone on high pole
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(-40, 13), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-26.75, 6.5, Math.toRadians(270)), Math.toRadians(270),
                                        slowVelocity, slowAccel)


                                //pick up third cone from pile
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-30.5, 10.5), Math.toRadians(135))
                                .splineToSplineHeading(new Pose2d(-35.5, 12, Math.toRadians(170)), Math.toRadians(170))
                                .splineToSplineHeading(new Pose2d(-64.5, 10, Math.toRadians(180)), Math.toRadians(180),
                                        slowVelocity, slowAccel)

                                //drop cone on medium pole
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(90)), Math.toRadians(25))
                                .splineToConstantHeading(new Vector2d(-25.25, 19), Math.toRadians(90))

                                //park
                                .setTangent(Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-12, 15), Math.toRadians(0))
                                .build()
                );


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setDimensions(14,14)
            .setDriveTrainType(DriveTrainType.MECANUM)

            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-32, 65, Math.toRadians(270)))
                        // t0
                        .lineTo(new Vector2d(23.5, 55.5))

                        // t1
                        .lineTo(new Vector2d(23.5, 63))

                        // t2
                        .setTangent(Math.toRadians(210))
                        .splineToSplineHeading(new Pose2d(12, 48, Math.toRadians(270)), Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(12, 24, Math.toRadians(270)), Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(35.5, 12, Math.toRadians(10)), Math.toRadians(10))

                        // t3
                        .splineTo(new Vector2d(64, 12), Math.toRadians(0),
                                slowVelocity, slowAccel)

                        // t4
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(40, 13), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(30, 6, Math.toRadians(270)), Math.toRadians(270),
                                slowVelocity, slowAccel)

                        // t5
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(35.5, 11, Math.toRadians(10)), Math.toRadians(10))

                        //t6
                        .splineTo(new Vector2d(63.5, 11), Math.toRadians(0),
                                slowVelocity, slowAccel)

                        // t7
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(28.5, 12, Math.toRadians(90)), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(31, 21.25), Math.toRadians(90))

                        // t8, position==1
                        .setTangent(Math.toRadians(260))
                        .splineToConstantHeading(new Vector2d(38, 11.5), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(63, 14, Math.toRadians((0))), Math.toRadians(0))

                        // t8, position==2
//                        .setTangent(Math.toRadians(270))
//                        .splineToConstantHeading(new Vector2d(37, 12), Math.toRadians(180))

                        // t8, position==3
//                        .setTangent(Math.toRadians(270))
//                        .splineToConstantHeading(new Vector2d(12.5, 12), Math.toRadians(180))

                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.85f)
                //.addEntity(myBot)
                .addEntity(myBotRight)
                .start();
    }
}