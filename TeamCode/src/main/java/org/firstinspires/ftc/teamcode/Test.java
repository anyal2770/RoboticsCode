package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleop.SliderTeleOp;

@TeleOp
public class Test extends LinearOpMode {
    private DcMotorEx FL, FR, BR, BL, intake;
    private DcMotorEx RSlides, LSlides;
    private double FRP, FLP, BLP, BRP;
    double x1;
    int targetPos = 1000, Rposition = 0, Lposition = 0;
    private double previousError1 = 0, error1 = 0, integralSum1 = 0, derivative1 = 0;
    private ElapsedTime lowerTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double Kp = 1.0, Kd = 0.00, Ki = 0.00, output = 0;//Don't use Ki
    double x, x2;
    private CRServo bucketCServo;
    private Servo bucketServo;

    @Override
    public void runOpMode() throws InterruptedException {
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        RSlides = hardwareMap.get(DcMotorEx.class, "RSlides");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        bucketCServo = hardwareMap.get(CRServo.class, "bucketCServo");

        BL.setDirection(DcMotorEx.Direction.REVERSE);//switched from BR TO BL
        FL.setDirection(DcMotorEx.Direction.REVERSE);//switched from FR TO FL
        FR.setDirection(DcMotorEx.Direction.FORWARD);//changed from DcMotorSimple
        BR.setDirection(DcMotorEx.Direction.FORWARD);
        RSlides.setDirection(DcMotorEx.Direction.FORWARD);
        LSlides.setDirection(DcMotorEx.Direction.REVERSE);

        RSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RSlides.setTargetPosition(0);
        LSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LSlides.setTargetPosition(0);
        LSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            this.driveT(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger);
            this.intakeCode(gamepad1.right_trigger, gamepad1.right_bumper);
            this.slides(gamepad2.right_trigger);
            this.Bucket(-gamepad2.left_stick_y);
            this.ContinuousBucket(-gamepad2.right_stick_y);
            telemetry.addData("Intake power", x1);
            telemetry.addData("RSlides Power", RSlides.getPower());
            telemetry.addData("LSlides Power", LSlides.getPower());
            telemetry.addData("Lslides pos", LSlides.getCurrentPosition());
            telemetry.addData("lSlides target", LSlides.getTargetPosition());
            telemetry.addData("rslide pos", RSlides.getCurrentPosition());
            telemetry.addData("rslide target", RSlides.getTargetPosition());
            telemetry.addData("output", output);
            telemetry.addData("bucketServo position", x);
            telemetry.addData("bucketCont power", x2);
            telemetry.update();

        }


    }

    public void driveT(double gamepadLY, double gamepadLX, double gamepadRX, double gamepadLT) {
        double y = 0.5 * (Math.pow(gamepadLY, 2)) * Math.signum(-gamepadLY); //y value is inver
        double x = 0.5 * (Math.pow(gamepadLX, 2)) * Math.signum(gamepadLX);
        double rx = -gamepadRX;
        double speedDivide = 1;
        if (gamepadLT > 0.3) {
            speedDivide = 2;
        } else {
            speedDivide = 1;
        }
        FLP = (y + x - rx) / speedDivide;
        FRP = (y - x + rx) / speedDivide;
        BLP = (y - x - rx) / speedDivide;
        BRP = (y + x + rx) / speedDivide;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FLP /= denominator;
        FRP /= denominator;
        BLP /= denominator;
        BRP /= denominator;

        FL.setPower(FLP);
        FR.setPower(FRP);
        BL.setPower(BLP);
        BR.setPower(BRP);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void intakeCode(double gamepadRT, boolean gamepadRB) {

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        if (gamepadRT > 0.3) {
            intake.setPower(1.0);
        } else if (gamepadRB == true) {
            intake.setPower(-0.5);
        } else {
            intake.setPower(0);
        }
        x1 = intake.getPower();
    }




    public void slides(double RT) {

        //Rposition = RSlides.getTargetPosition();
        // Lposition = LSlides.getTargetPosition();
        // boolean up = (y > 0.3);
        //boolean down = (y < -0.3);


        if (RT > 0.2) {
            targetPos = 1800;
            output = PIDControl1(targetPos, Rposition);
            RSlides.setPower(output * RT);
            LSlides.setPower(output * RT);
            LSlides.setTargetPosition(targetPos);
            RSlides.setTargetPosition(targetPos);
        } else if ((LSlides.getCurrentPosition() < 10 && LSlides.getCurrentPosition() > -10)) {
            RSlides.setPower(0);
            LSlides.setPower(0);
        } else {
            targetPos = 0;
            output = PIDControl1(targetPos, Rposition);
            RSlides.setPower(output * 0.5);
            LSlides.setPower(output * 0.5);
            LSlides.setTargetPosition(targetPos);
            RSlides.setTargetPosition(targetPos);

        }
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

    public void Bucket(double ry) {
        double position = 120;
        if (ry > 0.3 || ry < -0.3) {
            bucketServo.setPosition(position);
        } else {
            bucketServo.setPosition(0);
        }
        x = bucketServo.getPosition() * position;


    }

    public void ContinuousBucket(double ry) {
        if (ry > 0.3 || ry < -0.3) {
            bucketCServo.setPower(0.8 * ry);
        } else {
            bucketCServo.setPower(0);
        }
        x2 = bucketCServo.getPower();

    }
}


