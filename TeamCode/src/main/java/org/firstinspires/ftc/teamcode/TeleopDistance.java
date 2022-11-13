package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleopDistance", group = "18051")
public class TeleopDistance extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("sensor value side", drive.distanceSide.getDistance(DistanceUnit.INCH));
            telemetry.addData("sensor value forward", drive.distanceForward.getDistance(DistanceUnit.INCH));
            double[] val = drive.findTargetPosition();
            telemetry.addData("findPosition Forward", val[1]);
            telemetry.addData("findPosition Left", val[0]);
            telemetry.update();

        }
    }

}
