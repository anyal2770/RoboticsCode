package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous
public class FarAutonomousLeft1 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 2.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //private DcMotor FL   = null;  //  Used to control the left front drive wheel
    //private DcMotor FR  = null;  //  Used to control the right front drive wheel
    //private DcMotor BL    = null;  //  Used to control the left back drive wheel
    //private DcMotor BR   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 3;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private DcMotorEx FL, FR, BR, BL;
    private DcMotorEx RSlides, LSlides;
    int targetPos = 1000, Rposition = 0, Lposition = 0;
    private double previousError1 = 0, error1 = 0, integralSum1 = 0, derivative1 = 0;
    private ElapsedTime lowerTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double Kp = 1.0, Kd = 0.00, Ki = 0.00, output = 0;//Don't use Ki
    double x, x2;
    IMU imu, currYaw;
    //YawPitchRollAngles orientation;
    double currOrientation = 0;

    AngularVelocity angularVelocity;
    YawPitchRollAngles orientation;
    private CRServo bucketCServo, rubServo;
    private Servo bucketServo, openBucket;

    @Override public void runOpMode () {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        RSlides = hardwareMap.get(DcMotorEx.class, "RSlides");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        openBucket = hardwareMap.get(Servo.class, "openBucket");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        BL.setDirection(DcMotorEx.Direction.REVERSE);//switched from BR TO BL
        FL.setDirection(DcMotorEx.Direction.REVERSE);//switched from FR TO FL
        FR.setDirection(DcMotorEx.Direction.FORWARD);//changed from DcMotorSimple
        BR.setDirection(DcMotorEx.Direction.FORWARD);
        RSlides.setDirection(DcMotorEx.Direction.REVERSE);
        LSlides.setDirection(DcMotorEx.Direction.FORWARD);
        bucketServo.setDirection(Servo.Direction.REVERSE);
        // BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RSlides.setTargetPosition(0);
        LSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LSlides.setTargetPosition(0);
        LSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BL.setTargetPosition(0);
        BR.setTargetPosition(0);
        FL.setTargetPosition(0);
        FR.setTargetPosition(0);
        int targetPos1 = 500;
        int targetPos2 = 750;
        int targetPos3 = 500;


        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        bucketServo.setPosition(0.1);
        imu.resetYaw();
        if (opModeIsActive() && !isStopRequested()) {
            this.moveStraight(1400);
            sleep(500);
            this.rotate(70);
            //this.rotate(90);
            sleep(500);
            this.moveStraight(2200);
            sleep(500);
            this.rotate(50);
            while (opModeIsActive()) {
                targetFound = false;
                desiredTag = null;

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
                }

                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                if (targetFound) {

                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {

                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                    turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                    telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
                sleep(10);
                //if (desiredTag.ftcPose.range - DESIRED_DISTANCE < 2) {
                // this.slides();
                //}
            }
        }

    }



    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        FL.setPower(leftFrontPower);
        FR.setPower(rightFrontPower);
        BL.setPower(leftBackPower);
        BR.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    public void rotate(double x) {
        imu.resetYaw();

        orientation = imu.getRobotYawPitchRollAngles();
        double currentOrientation = orientation.getYaw(AngleUnit.DEGREES);
        //if(currentOrientation < -1){
        //  currentOrientation = 360 + currentOrientation;
        // }
        double targetOrientation = x;
        int targetPosFL = FL.getCurrentPosition();
        int targetPosBR = BR.getCurrentPosition();
        int targetPosFR = FR.getCurrentPosition();
        int targetPosBL = BL.getCurrentPosition();


        while (opModeIsActive() && Math.abs(targetOrientation - currentOrientation) > 1) {



            FL.setPower(-0.3 * Math.signum(x));
            BR.setPower(0.3 * Math.signum(x));
            BL.setPower(-0.3 * Math.signum(x));
            FR.setPower(0.3* Math.signum(x));
            //  FL.setTargetPosition(targetPosFL);
            // BR.setTargetPosition(targetPosBR);
            // BL.setTargetPosition(targetPosFL);
            //  BR.setTargetPosition(targetPosFR);
            // targetPosFL -= 2;
            // targetPosBR += 2;
            orientation = imu.getRobotYawPitchRollAngles();
            currentOrientation = orientation.getYaw(AngleUnit.DEGREES);
            // if(currentOrientation < -1){
            // currentOrientation = 360 + currentOrientation;
            //}

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Target yaw", targetOrientation);
            // telemetry.addData("FL current pos", FL.getCurrentPosition());
            // telemetry.addData("BR current pos", BR.getCurrentPosition());

            telemetry.update();
        }
    }

    public void moveStraight (int x) {
        int currentFL = FL.getCurrentPosition();
        int currentFR = FR.getCurrentPosition();
        int targetPosFL = currentFL + x;
        int targetPosFR = currentFR + x;
        while (opModeIsActive() && targetPosFL - currentFL > 0 && targetPosFR - currentFR > 0) {

            FL.setPower(0.5);
            FR.setPower(0.5);
            BR.setPower(0.5);
            BL.setPower(0.5);
            // FL.setTargetPosition(targetPosFL);
            //   FR.setTargetPosition(targetPosFR);

            currentFL = FL.getCurrentPosition();
            currentFR = FR.getCurrentPosition();


        }
    }
    public void slides() {
        targetPos = 1800;
        output = PIDControl1(targetPos, Rposition);
        RSlides.setPower( 0.3 * output);
        LSlides.setPower( 0.3 * output);
        LSlides.setTargetPosition(targetPos);
        RSlides.setTargetPosition(targetPos);
        sleep(500);
        openBucket.setPosition(1);
        sleep(1000);
        targetPos = 0;
        output = PIDControl1(targetPos, Rposition);
        RSlides.setPower(output * 0.3);
        LSlides.setPower(output * 0.3);
        LSlides.setTargetPosition(targetPos);
        RSlides.setTargetPosition(targetPos);

    }

    public double PIDControl1(double target, double state) {

        previousError1 = error1;
        error1 = target - state;
        if (error1 < 500 && error1 > -500) {

            integralSum1 += error1 * lowerTimer.seconds();
            derivative1 = (error1 - previousError1) / lowerTimer.seconds();

            lowerTimer.reset();
            double output = (error1 * Kp) / 500 + (derivative1 * Kd) + (integralSum1 * Ki);
            if (output > 1.0) {
                output = 1.0;
            }
            if (output < -1.0) {
                output = -1.0;
            }

        } else if (error1 > 500) {
            output = 1.0;
        } else {
            output = -1.0;
        }
        return output;

    }
}


