package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "leftSide", group = "18051")
public class Auto2 extends LinearOpMode {

    LlamaBot robot = new LlamaBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        robot.closeClaw(500);
        robot.armMoveToPosition(LlamaBot.ARM_POSITION_J1_DRIVE, this);
        //color sensor used here
        robot.strafeRightByTime(0.5, 1100);
        robot.driveForwardByTime(0.8,1000);
        robot.strafeLeftByTime(0.5, 650);
        robot.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DRIVE, this);
        robot.driveForwardByTime(0.3, 500);
        robot.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DROP,0.3, this);
        robot.openClaw(500);
        robot.driveForwardByTime(-0.5,200);
        robot.closeClaw(500);
        robot.strafeLeftByTime(0.5, 600);
        robot.driveForwardByTime(-0.5, 200);
        robot.armMoveToPosition(LlamaBot.ARM_POSITION_FLOOR, this);


    }
}


