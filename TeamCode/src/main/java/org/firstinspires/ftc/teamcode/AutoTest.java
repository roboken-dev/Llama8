package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoTest1", group = "18051")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-32, 65, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(18)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(trajectory1.end())
                .forward(2)
                .splineTo(new Vector2d(-12, 22), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-25.25, 13), Math.toRadians(270))
                .build();

        drive.closeClaw(500);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J1_DRIVE, this);
        drive.followTrajectory(trajectory1);
        int color = drive.getColor();
        telemetry.addData("color", color);
        telemetry.update();
        drive.followTrajectory(traj1);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DROP,0.3, this);

        double D1_TARGET = 3.8;
        double D2_TARGET = 11;
        double d1 = drive.distanceForward.getDistance(DistanceUnit.INCH);
        double d2 = drive.distanceSide.getDistance(DistanceUnit.INCH);
        double forwardDrive = d1 - D1_TARGET;
        double rightDrive = d2 - D2_TARGET;
        TrajectorySequence poleFinder = drive.trajectorySequenceBuilder(traj1.end())
                .setVelConstraint(new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH))
                //.strafeLeft(2)
                .forward(forwardDrive)
                .build();
/*
        while (!isStopRequested()) {
            telemetry.addData("sensor value side", drive.distanceSide.getDistance(DistanceUnit.INCH));
            telemetry.addData("sensor value forward", drive.distanceForward.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
*/


        drive.followTrajectorySequence(poleFinder);
        drive.openClaw(500);
/*
        telemetry.addData("Move back", "");
        telemetry.update();
        Trajectory back = drive.trajectoryBuilder(poleFinder.end())
                .forward(-6)
                .build();
        drive.followTrajectory(back);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J1_DRIVE, this);
        TrajectorySequence moveToFinalPos;
        color = LlamaBot.GREEN;
        if (color == LlamaBot.RED) {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .strafeLeft(10)
                    .forward(-10)
                    .build();
        } else if (color == LlamaBot.GREEN) {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .strafeRight(5)
                    .forward(-10)
                    .build();
        } else {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .strafeRight(27)
                    .forward(-10)
                    .build();

        }
        sleep(2000);
        telemetry.addData("Move to final position", color);
        telemetry.update();
        drive.followTrajectorySequence(moveToFinalPos);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_FLOOR, this);
        drive.closeClaw(500);

         */
    }
}
