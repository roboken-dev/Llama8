package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutoRightConesNP", group = "18051")
public class AutoRightConesNoPhoto extends LinearOpMode {

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    private static final String TFOD_MODEL_ASSET = "shapes-1.tflite";
    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };

    private static final String VUFORIA_KEY =
            "AXKY5gD/////AAABmaGYhDHFr0pZgCyH7lQCCpOIGDOsI8cBMvAUleX7dV8HGflhv2ztkJ2FmwOS9Pa3llYgoZ+qo+fpk+6WEwtLjcDyrEyr5pYjI/zdeCWkxB74tIRqbZJtps6kFgkAJRvOUq9Aoo3O0Ig6VnrFSaKTb/Y2V3kd2K3a6q6TvqJzB53dDyUrcHpTXn7WuIYc9DSSywgDYeQ6bL+SJBDBnF/J6qLZV1ELaUW6bP6ZY6MqtGw2yWnPU6WxgCpeCUFILt6a16Cggy7u3V6hLLvxp8cor0VYhuQqm4dLiR0iXI2FIXLK3zg8KgWpFz1UZKB4xH7deJQjMQCD5xJCJ3u+YBE2J22zVdiFBPAnRPj2sb19nKcP";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaTrackables targets = null;
    private WebcamName webcamName = null;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumVelocityConstraint slowVelocity = new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH);

        Pose2d startPose = new Pose2d(-32, 65, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        telemetry.addLine("Looking for target position");
        telemetry.update();

        // target position, we assume 3 if something fails...
        int targetPosition = 2;

        // Close claw on cone
        drive.closeClaw(400);

        Trajectory t0 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-24.5, 55))
                .build();
        Trajectory t1 = drive.trajectoryBuilder(t0.end())
                .lineTo(new Vector2d(-24.5, 63))
                .build();
        Trajectory t2 = drive.trajectoryBuilder(t1.end(), Math.toRadians(330))
                .splineToSplineHeading(new Pose2d(-12, 48, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-12, 24, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-35.5, 12, Math.toRadians(170)), Math.toRadians(170))
                .build();
        TrajectorySequence t3 = drive.trajectorySequenceBuilder(t2.end())
                .setVelConstraint(slowVelocity)
                .splineTo(new Vector2d(-64, 11.5), Math.toRadians(180))
                .build();
        Trajectory t4 = drive.trajectoryBuilder(t3.end(), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-40, 13), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-26.75, 6.5, Math.toRadians(270)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory t5 = drive.trajectoryBuilder(t4.end(), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30.5, 10.5), Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-35.5, 12, Math.toRadians(170)), Math.toRadians(170))
                .build();
        Trajectory t6 = drive.trajectoryBuilder(t5.end(), Math.toRadians(190))
                .splineToSplineHeading(new Pose2d(-64.5, 10, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory t7 = drive.trajectoryBuilder(t6.end(), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(90)), Math.toRadians(25))
                .splineToConstantHeading(new Vector2d(-25.25, 19), Math.toRadians(90))
                .build();
        Trajectory t8;
        if (targetPosition == 1) {
            t8 = drive.trajectoryBuilder(t7.end(), Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(-12, 15), Math.toRadians(0))
                    .build();
        } else if (targetPosition == 2) {
            t8 = drive.trajectoryBuilder(t7.end(), Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(-35.50, 12.25), Math.toRadians(90))
                    .build();
        } else {
            t8 = drive.trajectoryBuilder(t7.end(), Math.toRadians(280))
                    .splineToLinearHeading(new Pose2d(-60.5, 12.5, Math.toRadians(180)), Math.toRadians(180))
                    .build();
        }

        // Try to read sleeve position
        int count = 0;
        while (!isStarted()) {
            ++count;
            if (tfod == null) {
                // Initialization failed, we cannot use camera, just continue with default targetPosition
                break;
            }

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions == null) {
                telemetry.addLine("no recognitions - null");
                targetPosition = 2;
            } else if (updatedRecognitions.size() == 0) {
                telemetry.addLine("no recognitions - 0");
                targetPosition = 2;
            } else {
                telemetry.addData("recognitions", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                Recognition selected = null;
                for (Recognition recognition : updatedRecognitions) {
                    if (selected == null || recognition.getConfidence() > selected.getConfidence()) {
                        selected = recognition;
                    }
                }
                String label = selected.getLabel();
                if (label.equals("1")) {
                    targetPosition = 1;
                } else if (label.equals("2")) {
                    targetPosition = 2;
                } else if (label.equals("3")) {
                    targetPosition = 3;
                }
            }
            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("count", count);
            telemetry.addLine("Ready to start...");
            telemetry.update();
        }

        // Need this in case tfod==null
        waitForStart();
        if (isStopRequested()) return;

        // Pick cone to driving height
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J2_DRIVE, 1.0, false, this);

        // Move to low pole and drop cone
        drive.followTrajectory(t0);
        if (isStopRequested()) return;
        drive.openClaw(200);

        // Move backwards, lower arm, and move to check position
        drive.followTrajectory(t1);
        if (isStopRequested()) return;

        drive.armMoveToPosition(LlamaBot.ARM_POSITION_CONE_PICK1, 1.0, false, this);
        drive.followTrajectory(t2);
        if (isStopRequested()) return;

        // Move to cone pile and pick up cone
        drive.followTrajectorySequence(t3);
        if (isStopRequested()) return;
        drive.closeClaw(300);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DRIVE, 1.0, false, this);
        sleep(200);

        // Go to high pole and drop cone
        drive.followTrajectory(t4);
        if (isStopRequested()) return;
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J4_DROP, 0.5, this);
        drive.openClaw(150);

        // Move to check position
        drive.followTrajectory(t5);
        if (isStopRequested()) return;

        // Lower arm to pick 2nd cone from pile
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_CONE_PICK2, 1.0, false, this);

        // Move to cone pile and pick up 2nd cone
        drive.followTrajectory(t6);
        if (isStopRequested()) return;
        drive.closeClaw(300);
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J3_DRIVE, 1.0, false, this);
        sleep(200);

        // Move to medium pole and drop cone
        drive.followTrajectory(t7);
        if (isStopRequested()) return;
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_J3_DROP, 0.5, this);
        drive.openClaw(150);

        // Move to targetPosition
        drive.followTrajectory(t8);
        telemetry.addData("pose", drive.getPoseEstimate());
        telemetry.update();
        drive.armMoveToPosition(LlamaBot.ARM_POSITION_FLOOR, 1.0, true, this);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targets);

        // Name and locate each trackable object
        identifyTarget(0, "Red Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Red Rear Wall", halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Blue Rear Wall", halfField, oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = 2.25f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 7.25f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = -5.375f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setClippingMargins(230, 150, 230, 150);
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    boolean updatePosition(SampleMecanumDrive drive) {
        boolean targetVisible = false;
        Pose2d newPose = null;

        targets.activate();
        sleep(200);
        int count = 0;
        while (!targetVisible && count < 100) {
            ++count;

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addLine(trackable.getName());
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                newPose = new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, Math.toRadians(rotation.thirdAngle));

                telemetry.addData("Pos", newPose);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.addData("count", count);
            telemetry.addData("Pose Estimate", drive.getPoseEstimate());
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targets.deactivate();
        if (newPose != null) {
            drive.setPoseEstimate(newPose);
            telemetry.addData("New Pose Estimate", drive.getPoseEstimate());
            telemetry.update();
            return true;
        }
        return false;
    }
}
