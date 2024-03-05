package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OnePlayer extends LinearOpMode {
    private DcMotorEx FL, FR, BR, BL, RSlides, LSlides;
    private double FRP, FLP, BLP, BRP;
    double x1;

    @Override
    public void runOpMode() throws InterruptedException {
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        RSlides = hardwareMap.get(DcMotorEx.class, "RSlides");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");

        BL.setDirection(DcMotorEx.Direction.REVERSE);//switched from BR TO BL
        FL.setDirection(DcMotorEx.Direction.REVERSE);//switched from FR TO FL
        FR.setDirection(DcMotorEx.Direction.FORWARD);//changed from DcMotorSimple
        BR.setDirection(DcMotorEx.Direction.FORWARD);
        RSlides.setDirection(DcMotorEx.Direction.FORWARD);
        LSlides.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            this.driveT(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.right_bumper);
            this.slides(gamepad2.left_stick_y);
            telemetry.addData("power", x1);
            telemetry.update();

        }


    }
    public void driveT(double gamepadLY, double gamepadLX, double gamepadRX, double gamepadLT, double gamepadRT, boolean gamepadRB){
        double y = 0.5*(Math.pow(gamepadLY,2))*Math.signum(-gamepadLY); //y value is inverted
        double x =0.5*(Math.pow(gamepadLX,2))*Math.signum(gamepadLX);
        double rx = -gamepadRX;
        double speedDivide = 1;
        if(gamepadLT > 0.3){
            speedDivide = 2;
        }
        else{
            speedDivide = 1;
        }
        FLP = (y + x - rx)/speedDivide;
        FRP = (y - x + rx)/speedDivide;
        BLP = (y - x - rx)/speedDivide;
        BRP = (y + x + rx)/speedDivide;

        double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
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



        DcMotorEx Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setDirection(DcMotorEx.Direction.FORWARD);
        if (gamepadRT>0.3){
            Intake.setPower(1.0);
        } else if (gamepadRB == true) {
            Intake.setPower(0.5);
        } else {
            Intake.setPower(0);
        }
        x1= Intake.getPower();
    }

    public void slides(double y){
        RSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if (y > 0.3 || y < 0.3) {
            RSlides.setPower(-y);
            LSlides.setPower(-y);

        } else {
            RSlides.setPower(0.0);
            LSlides.setPower(0.0);
        }

    }

}

