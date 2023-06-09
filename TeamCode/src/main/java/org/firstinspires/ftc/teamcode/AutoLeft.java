package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoLeft", group = "18051")
@Disabled
public class AutoLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(40, 65, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(18)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(trajectory1.end())
                .forward(2)
                .splineTo(new Vector2d(60, 22), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(22.75, 13), Math.toRadians(270))
                .build();

        drive.closeClaw(500);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J1_DRIVE, this);
        drive.followTrajectory(trajectory1);
        int color = drive.getColorRed();
        telemetry.addData("color", color);
        telemetry.update();
        drive.followTrajectory(traj1);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DROP,0.3, this);

        double D1_TARGET = 4.1;
        double D2_TARGET = 11;
        double d1 = drive.distanceForward.getDistance(DistanceUnit.INCH);
//        double d2 = drive.distanceSide.getDistance(DistanceUnit.INCH);
        double forwardDrive = d1 - D1_TARGET;
        if (forwardDrive > 5) {
            forwardDrive = 5;
        } else if (forwardDrive < -5) {
            forwardDrive = -2;
        }
        telemetry.addData("forward drive", forwardDrive);
        telemetry.addData("d1", d1);
//        telemetry.addData("d2", d2);
        telemetry.update();
//        double rightDrive = D2_TARGET - d2;
        TrajectorySequence poleFinder = drive.trajectorySequenceBuilder(traj1.end())
                .setVelConstraint(new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH))
                // .strafeLeft(2)
                .forward(forwardDrive)
                .build();
        drive.followTrajectorySequence(poleFinder);
        drive.openClaw(500);
        Trajectory back = drive.trajectoryBuilder(poleFinder.end())
                .forward(-6)
                .build();
        drive.followTrajectory(back);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J1_DRIVE, this);
        TrajectorySequence moveToFinalPos;
        if (color == LlamaBot.RED) {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .strafeLeft(34)
                    .forward(-10)
                    .build();
        } else if (color == LlamaBot.GREEN) {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .strafeLeft(10)
                    .forward(-10)
                    .build();
        } else {
            moveToFinalPos = drive.trajectorySequenceBuilder(back.end())
                    .strafeRight(10)
                    .forward(-10)
                    .build();

        }
        drive.followTrajectorySequence(moveToFinalPos);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_FLOOR, this);
        drive.closeClaw(500);
    }
}
