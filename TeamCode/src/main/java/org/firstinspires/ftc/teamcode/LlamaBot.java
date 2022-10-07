package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class LlamaBot
{
    static final int ARM_POSITION_FLOOR = 0;
    static final int ARM_POSITION_L1_DROP = 420;
    static final int ARM_POSITION_L1_DRIVE = 510;
    static final int ARM_POSITION_L2_DROP = 1210;
    static final int ARM_POSITION_L2_DRIVE = 1310;
    static final int ARM_POSITION_L3_DROP = 2090;
    static final int ARM_POSITION_L3_DRIVE = 2240;
    static final int ARM_POSITION_TOP = 2500;
    static final int ELEMENT_THRESHHOLD = 25;
    public DcMotor motorFrontLeft;  // motor1
    public DcMotor motorRearLeft;  // motor 2
    public DcMotor motorFrontRight; // motor 3
    public DcMotor motorRearRight; // motor 4
    public Servo claw;
    public DcMotor arm;
   // public DistanceSensor distance;


    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU               imu;  //Note: you must configure the IMU on I2C channel 0, port 0.
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    double                  clawPosition;


    static final double     SCALE_FACTOR = 75.0/75.0; //  if drive speed = .2 or .3 use 75.0/75.0;  .5 is 75.0/76.0 .4 is 75.0/75.5 if drive_speed = .1, use 1.0; if drive_speed = .3, use 75.0/77.0 note that .3 has hard time braking
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: GoBuildaYellowJacket 312 RPM Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77952756 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (SCALE_FACTOR * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     Arm_Speed               = 0.3;
    static final int        ARM_BOTTOM              = ARM_POSITION_FLOOR;
    static final int        ARM_TOP                 = ARM_POSITION_TOP;
    static final double     ARM_SPEED               = 0.5;
    static final double     CLAW_OPEN               = 1;
    static final double     CLAW_CLOSE              = 0.3;
    static final double     CLAW_STEP               = 0.05;



    HardwareMap hwMap = null;
    private ElapsedTime period =new ElapsedTime();

    public LlamaBot() { }


    public void init(HardwareMap hwMap, LinearOpMode opmode) {

        this.hwMap = hwMap;
        arm = hwMap.dcMotor.get("arm");
        claw = hwMap.servo.get("claw");
   //     distance = hwMap.get(DistanceSensor.class, "distance");

        // reset to default
        initMotors();
        initRunWithoutEncoder();

        initIMU(opmode);

        resetAngle(); // reset robot IMU angle to current heading. Zero.

        armInit();

        opmode.telemetry.addData("Status", "Ready for Start");
        opmode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opmode.telemetry.update();
    }

    public void initIMU(LinearOpMode opmode) {
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        opmode.telemetry.addData("Status", "Calibrating IMU...");
        opmode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated())
        {
            opmode.sleep(50);
            opmode.idle();
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, LinearOpMode opmode) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Send telemetry message to signify robot waiting;
        opmode.telemetry.addData("Status", "Resetting Encoders");    //
        opmode.telemetry.update();

        initRunWithEncoder();

        // Send telemetry message to indicate successful Encoder reset
        opmode.telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorRearLeft.getCurrentPosition(),
                motorRearRight.getCurrentPosition());
        opmode.telemetry.update();

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newFrontRightTarget = (motorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
            newRearLeftTarget = (motorRearLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRearRightTarget = (motorRearRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));

            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);
            motorFrontLeft.setPower(speed);
            motorFrontRight.setPower(speed);
            motorRearLeft.setPower(speed);
            motorRearRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearLeft.isBusy() && motorRearRight.isBusy())) {

                // Display it for the driver.
                opmode.telemetry.addData("Path1",  "Running to %7d :%7d : %d : %d", newFrontLeftTarget,  newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                opmode.telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition());
                opmode.telemetry.update();

            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorRearLeft.setPower(0);
            motorRearRight.setPower(0);

            initRunWithoutEncoder();
            // Turn off RUN_TO_POSITION

        }
    }

    public void initMotors() {
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorRearLeft = hwMap.dcMotor.get("motorRearLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorRearRight = hwMap.dcMotor.get("motorRearRight");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void initRunWithEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initRunWithoutEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void driveForward(double power) {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorRearLeft.setPower(power);
        motorRearRight.setPower(power);
    }

    public void driveBackward(double power) {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorRearLeft.setPower(-power);
        motorRearRight.setPower(-power);
    }

    public void stopDriving() {
        driveForward(0);
    }

    public float getCurrentZ() {
        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return o.firstAngle;
    }

    public void turn(double power, float angle, LinearOpMode opmode) {
        if (angle < -180 || angle > 180) {
            // Invalid angle provided!!!
            return;
        }
        float startPos = getCurrentZ();
        if (angle < 0) {
            turnRight(power);
            /*
            while ((getCurrentZ() - startPos) > angle) {
                opmode.idle();
            }
             */
            while (true) {
                float delta = (getCurrentZ() - startPos) - angle;
//                opmode.telemetry.addData("delta", delta);
//                opmode.telemetry.addData("currZ", getCurrentZ());
//                opmode.telemetry.addData("startPos", startPos);
//                opmode.telemetry.addData("angle", angle);
//                opmode.telemetry.update();
                if (delta < 1) {
                    break;
                }
                if (delta <= 20 && power > 0.3) {
                    turnRight(0.3);
                }
            }
            turnRight(0);
        } else {
            turnLeft(power);
            /*
            while ((getCurrentZ() - startPos) < angle) {
                opmode.idle();
            }
             */
            while (true) {
                float delta = angle - (getCurrentZ() - startPos);
                if (delta < 1) {
                    break;
                }
                if (delta <= 10 && power > 0.3) {
                    turnRight(0.3);
                }
            }
            turnLeft(0);
        }
    }

    public void turnLeft(double power) {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorRearLeft.setPower(-power);
        motorRearRight.setPower(power);
    }

    public void turnRight(double power) {
        turnLeft(-power);
    }

    public void driveForwardByTime(double power, long time) throws InterruptedException {
        driveForward(power);
        Thread.sleep(time);
        stopDriving();

    }

    public void turnLeftByTime(double power, long time) throws InterruptedException {
        turnLeft(power);
        Thread.sleep(time);
        stopDriving();

    }

    public void turnRightByTime(double power, long time) throws InterruptedException {
        turnRight(power);
        Thread.sleep(time);
        stopDriving();
    }

    public void turnByTime(boolean turnRight, double power, long time) throws InterruptedException {
        if (turnRight) {
            turnRight(power);
        } else {
            turnLeft(power);
        }
        Thread.sleep(time);
        stopDriving();
    }

    public void strafeRight(double power)
    {
        motorFrontLeft.setPower(power);
        motorRearLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorRearRight.setPower(power);
    }

    public void strafeLeft(double power) {
        motorFrontLeft.setPower(-power);
        motorRearLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorRearRight.setPower(-power);
    }

    public void strafeLeftByTime(double power, long time) throws InterruptedException {
        strafeLeft(power);
        Thread.sleep(time);
        stopDriving();
    }

    public void strafeRightByTime(double power, long time) throws InterruptedException {
        strafeRight(power);
        Thread.sleep(time);
        stopDriving();
    }

    public void driveDiagonal(boolean forward, boolean right, double power) {
        if (forward && right) {
            // Forward & Right
            motorFrontLeft.setPower(power);
            motorRearRight.setPower(power);
        } else if (forward) {
            // Forward & Left
            motorRearLeft.setPower(power);
            motorFrontRight.setPower(power);
        } else if (right) {
            // Backward & Right
            motorRearLeft.setPower(-power);
            motorFrontRight.setPower(-power);
        } else {
            // Backward & Left
            motorFrontLeft.setPower(-power);
            motorRearRight.setPower(-power);
        }
    }

    public void driveDiagonalByTime(boolean forward, boolean right, double power, long time) throws InterruptedException {
        driveDiagonal(forward, right, power);
        Thread.sleep(time);
        stopDriving();
    }

    public void clawInit(double position) {
        clawPosition = position;
        claw.setPosition(clawPosition);
    }

    public void openClaw(int timeout) throws InterruptedException {
        clawPosition = CLAW_OPEN;
        claw.setPosition(clawPosition);
        Thread.sleep(timeout);
    }

    public void closeClaw(int timeout) throws InterruptedException {
        clawPosition = CLAW_CLOSE;
        claw.setPosition(clawPosition);
        Thread.sleep(timeout);
    }

    public void openClawStep() throws InterruptedException {
        clawPosition += CLAW_STEP;
        if (clawPosition > CLAW_OPEN) {
            clawPosition = CLAW_OPEN;
        }
        claw.setPosition(clawPosition);
        Thread.sleep(50);
    }

    public void closeClawStep() throws InterruptedException {
        clawPosition -= CLAW_STEP;
        if (clawPosition < CLAW_CLOSE) {
            clawPosition = CLAW_CLOSE;
        }
        claw.setPosition(clawPosition);
        Thread.sleep(50);
    }

    public void armInit() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void armMoveToPosition(int position, LinearOpMode opmode) {
        arm.setTargetPosition(position);
        arm.setPower(0.7);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.getCurrentPosition() != position) {
            opmode.idle();
        }
    }





    //Resets the cumulative angle tracking to zero.
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }



    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power, boolean retainCurrentAngle, LinearOpMode opmode)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        if(retainCurrentAngle==false) {
            resetAngle();
        }

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            turnRight(power);
        }
        else if (degrees > 0)
        {   // turn left.
            turnLeft(power);
        }
        else return;

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opmode.opModeIsActive() && getAngle() == 0) {}

            while (opmode.opModeIsActive() && getAngle() > degrees) {
                double currentAngle = getAngle();

                opmode.telemetry.addData("angle is ",String.valueOf(currentAngle));
                opmode.telemetry.update();
            }
        }
        else    // left turn.
            while (opmode.opModeIsActive() && getAngle() < degrees) {
                double currentAngle = getAngle();

                opmode.telemetry.addData("angle is ",String.valueOf(currentAngle));
                opmode.telemetry.update();

            }

        // turn the motors off.

        stopDriving();

        // wait for rotation to stop.
        opmode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();

    }
}


