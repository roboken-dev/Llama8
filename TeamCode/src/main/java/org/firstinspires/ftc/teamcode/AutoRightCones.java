package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoRightCones", group = "18051")
public class AutoRightCones extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumVelocityConstraint slowVelocity = new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH);

        Pose2d startPose = new Pose2d(-32, 65, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        // Pick cone to driving height
        drive.closeClaw(500);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J2_DRIVE, this);

        // Move to Low Pole #1
        /* ORIG:
        Trajectory moveToLowPole1 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-16, 49.25, Math.toRadians(180)), Math.toRadians(180))
                .build();
        drive.followTrajectory(moveToLowPole1);
         */
        Trajectory moveToLowPole1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-24.5, 56))
                .build();
        drive.followTrajectory(moveToLowPole1);

        // Move to drop cone position
        double TARGET_DISTANCE = 5;
        double distance = drive.distanceForward.getDistance(DistanceUnit.INCH);
        telemetry.addData("distance", distance);
        telemetry.update();
        double forwardDrive = distance - TARGET_DISTANCE;
        sleep(2000);
        if (forwardDrive > 5) {
            forwardDrive = 5;
        } else if (forwardDrive < -5) {
            forwardDrive = -2;
        }

        TrajectorySequence moveToDropCone = drive.trajectorySequenceBuilder(moveToLowPole1.end())
                .setVelConstraint(new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH))
                .forward(forwardDrive)
                .build();
        drive.followTrajectorySequence(moveToDropCone);

        // Drop Cone
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J2_DROP,0.3, this);
        drive.openClaw(500);

        // Move to cones pile
        /* ORIG:
        Trajectory moveToPile = drive.trajectoryBuilder(moveToDropCone.end())
                .splineToConstantHeading(new Vector2d(-12, 49.25), Math.toRadians(315))
                .splineToConstantHeading(new Vector2d(-12, 26), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-65, 11.25), Math.toRadians(180))
                .build();
        drive.followTrajectory(moveToPile);
         */
        Trajectory moveToPile = drive.trajectoryBuilder(moveToDropCone.end())
                .splineToConstantHeading(new Vector2d(-22, 65), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-12, 26), Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(-62, 11.25, Math.toRadians(180)), Math.toRadians(180))
                .splineTo(new Vector2d(-56, 11.25), Math.toRadians(180))
                .build();
        drive.followTrajectory(moveToPile);

        // Move to cone pick up position
        TARGET_DISTANCE = 5;
        distance = drive.distanceForward.getDistance(DistanceUnit.INCH);
        forwardDrive = distance - TARGET_DISTANCE;
        if (forwardDrive > 5) {
            forwardDrive = 5;
        } else if (forwardDrive < -5) {
            forwardDrive = -2;
        }
        telemetry.addData("distance", distance);
        telemetry.update();
        sleep(10000);
        TrajectorySequence moveToPickCone = drive.trajectorySequenceBuilder(moveToPile.end())
                .setVelConstraint(slowVelocity)
                .forward(forwardDrive)
                .build();
        drive.followTrajectorySequence(moveToPickCone);

        // Pick up cone
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_CONE_PICK1, 0.7, this);
        drive.closeClaw(500);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DRIVE, 0.7, this);

        // Move to high pole
        Trajectory moveToHighPole = drive.trajectoryBuilder(moveToPickCone.end())
                .splineToSplineHeading(new Pose2d(-25.25, 7, Math.toRadians(0)), Math.toRadians(270))
                .build();
        drive.followTrajectory(moveToHighPole);

        // Move to drop cone position
        TARGET_DISTANCE = 11;
        distance = drive.distanceForward.getDistance(DistanceUnit.INCH);
        forwardDrive = distance - TARGET_DISTANCE;
        telemetry.addData("distance", distance);
        telemetry.update();
        sleep(2000);
        if (forwardDrive > 5) {
            forwardDrive = 5;
        } else if (forwardDrive < -5) {
            forwardDrive = -2;
        }
        TrajectorySequence moveToDropCone2 = drive.trajectorySequenceBuilder(moveToHighPole.end())
                .setVelConstraint(slowVelocity)
                .forward(forwardDrive)
                .build();
        drive.followTrajectorySequence(moveToDropCone2);

        // Drop cone
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DROP, 0.3, this);
        drive.openClaw(500);

        // Move to cone pile
        Trajectory moveToPile2 = drive.trajectoryBuilder(moveToDropCone2.end())
                .splineTo(new Vector2d(-28, 16), Math.toRadians(135))
//                .splineToSplineHeading(new Pose2d(-56, 11.25, Math.toRadians(180)), Math.toRadians(180))
                .splineTo(new Vector2d(-56, 11.25), Math.toRadians(180))
                .build();
        drive.followTrajectory(moveToPile2);

        // Move to cone pick up position
        TARGET_DISTANCE = 11;
        distance = drive.distanceForward.getDistance(DistanceUnit.INCH);
        forwardDrive = distance - TARGET_DISTANCE;
        if (forwardDrive > 5) {
            forwardDrive = 5;
        } else if (forwardDrive < -5) {
            forwardDrive = -2;
        }
        TrajectorySequence moveToPickCone2 = drive.trajectorySequenceBuilder(moveToPile.end())
                .setVelConstraint(slowVelocity)
                .forward(forwardDrive)
                .build();
        drive.followTrajectorySequence(moveToPickCone2);

        // Pick up cone
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_CONE_PICK2,0.7, this);
        drive.closeClaw(500);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J2_DRIVE, 0.7, this);

        // Move to low pole #2
        Trajectory moveToLowPole2 = drive.trajectoryBuilder(moveToPickCone2.end())
                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(0)), Math.toRadians(90))
                .build();
        drive.followTrajectory(moveToLowPole2);

        // Move to drop cone position
        TARGET_DISTANCE = 11;
        distance = drive.distanceForward.getDistance(DistanceUnit.INCH);
        forwardDrive = distance - TARGET_DISTANCE;
        if (forwardDrive > 5) {
            forwardDrive = 5;
        } else if (forwardDrive < -5) {
            forwardDrive = -2;
        }
        TrajectorySequence moveToDropCone3 = drive.trajectorySequenceBuilder(moveToLowPole2.end())
                .setVelConstraint(slowVelocity)
                .forward(forwardDrive)
                .build();
        drive.followTrajectorySequence(moveToDropCone3);

        // Drop cone
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J2_DROP, 0.3, this);
        drive.openClaw(500);

        // Move to end location
        // TODO
        /*
        TrajectorySequence moveToFinalPos;
        if (color == LlamaBot.RED) {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .splineToConstantHeading(new Vector2d (-12,10),Math.toRadians(5))
                    .build();
        } else if (color == LlamaBot.GREEN) {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .splineToConstantHeading(new Vector2d (-36,10), Math.toRadians(30))
                    .build();
        } else {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .splineToConstantHeading(new Vector2d (-60,10), Math.toRadians(180))
                    .build();

        }
        drive.followTrajectorySequence(moveToFinalPos);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_FLOOR, this);
        drive.closeClaw(500);

         */
    }
}










