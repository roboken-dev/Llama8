package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@TeleOp(name = "TeleOp", group = "12806")
public class Teleop extends LinearOpMode {

    LlamaBot robot   = new LlamaBot();

    @Override
    public void runOpMode() throws InterruptedException {
        double speed_control = 0.5;
        double ArmSpeedControl = 0.6;


        robot.init(hardwareMap,this);
/*
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
*/
        robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRearLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.motorRearRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to Go");    //
        telemetry.update();

        waitForStart();

        robot.arm.setTargetPosition(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.clawInit(robot.CLAW_CLOSE);
        while (opModeIsActive()) {

            double G1rightStickY = gamepad1.right_stick_y;
            double G1leftStickY = gamepad1.left_stick_y;
            double armSpeed = -gamepad2.left_stick_y * robot.ARM_SPEED;
            float G1rightTrigger = gamepad1.right_trigger;
            float G1leftTrigger = gamepad1.left_trigger;
            float G2leftTrigger = gamepad2.left_trigger;
            float G2rightTrigger = gamepad2.right_trigger;

            if (armSpeed < 0) {
                if (robot.arm.getTargetPosition() != robot.ARM_BOTTOM) {
                    robot.arm.setTargetPosition(robot.ARM_BOTTOM);
                }
            } else if (armSpeed > 0) {
                if (robot.arm.getTargetPosition() != robot.ARM_TOP) {
                    robot.arm.setTargetPosition(robot.ARM_TOP);
                }
            }

            if (gamepad2.x) {
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                boolean exitLoop = false;
                while (opModeIsActive() && !exitLoop) {
                    if (gamepad2.y) {
                        exitLoop = true;
                    }
                    double innerArmSpeed = -gamepad2.left_stick_y * robot.ARM_SPEED * 0.5;
                    robot.arm.setPower(innerArmSpeed);
                }
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setTargetPosition(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("Arm Position", robot.arm.getCurrentPosition());

            robot.arm.setPower(armSpeed);

            //Driver 1 wheel speed control
            if (gamepad1.dpad_up) {
                speed_control = 1;
                telemetry.addData("Status", "Setting Speed to 1");    //
            }
            if (gamepad1.dpad_down) {
                speed_control = 0.25f;
                telemetry.addData("Status", "Setting Speed to .25");    //
            }
            if (gamepad1.dpad_left) {
                speed_control = 0.5f;
                telemetry.addData("Status", "Setting Speed to .5");    //
            }
            if (gamepad1.dpad_right) {
                speed_control = 0.5f;
                telemetry.addData("Status", "Setting Speed to .5");    //
            }

            if (G1rightTrigger > 0 && G1leftTrigger == 0) {

                //Strafe Right
                robot.motorFrontLeft.setPower(-G1rightTrigger * speed_control);
                robot.motorRearLeft.setPower(G1rightTrigger * speed_control);
                robot.motorFrontRight.setPower(G1rightTrigger * speed_control);
                robot.motorRearRight.setPower(-G1rightTrigger * speed_control);

                telemetry.addData("Status", "Strafing Right");    //
            } else if (G1leftTrigger > 0 && G1rightTrigger == 0) {

                //Strafe Left
                robot.motorFrontLeft.setPower(G1leftTrigger * speed_control);
                robot.motorRearLeft.setPower(-G1leftTrigger * speed_control);
                robot.motorFrontRight.setPower(-G1leftTrigger * speed_control);
                robot.motorRearRight.setPower(G1leftTrigger * speed_control);

                telemetry.addData("Status", "Strafing Left");    //
            } else {

                //Tank Drive
                robot.motorFrontLeft.setPower(G1leftStickY * Math.abs(G1leftStickY) * speed_control);
                robot.motorRearLeft.setPower(G1leftStickY * Math.abs(G1leftStickY) * speed_control);
                robot.motorFrontRight.setPower(G1rightStickY * Math.abs(G1rightStickY) * speed_control);
                robot.motorRearRight.setPower(G1rightStickY * Math.abs(G1rightStickY) * speed_control);

                telemetry.addData("Status", "Moving");    //
            }

            if (gamepad2.right_bumper) {
                robot.closeClawStep();
            } else if (gamepad2.left_bumper) {
                robot.openClawStep();
            }


            if (G2leftTrigger > 0) {
                robot.spinner.setPower(0.7 * G2leftTrigger);
            } else if (G2rightTrigger > 0) {
                robot.spinner.setPower(-0.7 * G2rightTrigger);
            } else {
                robot.spinner.setPower(0);
            }

            telemetry.addData("Distance Sensor", robot.distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Z absolute", robot.getCurrentZ());
            telemetry.addData("encoders",
                    "FL=%d RL=%d FR=%d RR=%d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorRearLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition(),
                    robot.motorRearRight.getCurrentPosition());
            telemetry.update();
        }
    }
}


// old code


/*

            if (gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0) {
                robot.arm.setPower(gamepad2.left_stick_y * ArmSpeedControl+0.2);
            }

            if (gamepad2.left_stick_y < 0) {
                robot.arm.setPower(gamepad2.left_stick_y *( ArmSpeedControl));
            }

            if (gamepad2.left_stick_y == 0) {
                robot.arm.setPower(0.0);

            }

            if (gamepad2.y)
            {
                ArmSpeedControl = 1.0;
            }


            if (gamepad2.a)
            {
                ArmSpeedControl = 0.4;
            }
            if (gamepad2.b)
            {
                ArmSpeedControl = 0.8;
            }

            if (G2leftTrigger > 0) {
                robot.spinner.setPower(0.6);
            } else if (G2rightTrigger > 0) {
                robot.spinner.setPower(-0.6);
            } else {
                robot.spinner.setPower(0);
            }

        }

        // Set the panel back to the default color
        /*
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }

         */
