package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.teamcode.Teleop.SliderTeleOp;

@TeleOp
public class driveTrain extends LinearOpMode {

    private DcMotorEx FL, FR, BR, BL;
    private double FRP, FLP, BLP, BRP;
    double x1;

    @Override
    public void runOpMode() throws InterruptedException {
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        double speedDivide = 1;
        BL.setDirection(DcMotorEx.Direction.REVERSE);//switched from BR TO BL
        FL.setDirection(DcMotorEx.Direction.REVERSE);//switched from FR TO FL
        FR.setDirection(DcMotorEx.Direction.FORWARD);//changed from DcMotorSimple
        BR.setDirection(DcMotorEx.Direction.FORWARD);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            //this.driveTrain(double gLY, double gLX, double gRS, double gLT, double gRT)
            double y = 0.5*(Math.pow(-gamepad1.left_stick_y,2))*Math.signum(-gamepad1.left_stick_y); //y value is inverted
            double x =0.5*(Math.pow(gamepad1.left_stick_x,2))*Math.signum(gamepad1.left_stick_x);
            double rx = -gamepad1.right_stick_x;

            if(gamepad1.left_trigger > 0.3){
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

            /*DcMotorEx Intake = hardwareMap.get(DcMotorEx.class, "Intake");
            Intake.setDirection(DcMotorEx.Direction.FORWARD);
            if (gamepad1.right_trigger>0.3){
                Intake.setPower(1.0);
            } else if (gamepad1.right_bumper == true) {
                Intake.setPower(0.5);
            } else {
                Intake.setPower(0);
            }
            x1= Intake.getPower();
            telemetry.addData("power", x1);
            telemetry.update();
*/
        }
        //SliderTeleOp sliders = new SliderTeleOp();

    }




}






