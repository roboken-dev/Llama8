package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleopDistance", group = "18051")
public class TeleopDistance extends LinearOpMode {

    public DistanceSensor distance1;
    public DistanceSensor distance2;




    @Override
    public void runOpMode() throws InterruptedException {
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("sensor value", distance1.getDistance(DistanceUnit.CM));
            telemetry.addData("sensor value", distance2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}
