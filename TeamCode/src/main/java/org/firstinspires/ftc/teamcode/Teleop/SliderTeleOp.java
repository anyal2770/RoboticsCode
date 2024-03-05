package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SliderTeleOp extends LinearOpMode {


    private DcMotorEx RSlides, LSlides;
    private CRServo bucketCServo;
    private Servo bucketServo;
    int targetPos = 1500;
    int Rposition = 0;
    int Lposition = 0;
    private double previousError1 = 0, error1 = 0, integralSum1 =0, derivative1 = 0;
    private ElapsedTime lowerTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double Kp = 1.00, Kd = 0.000, Ki = 0.00, output = 0;//Don't use Ki

    double x, x2;
    //CRServo Bucket = hardwareMap.get(CRServo.class, "bucket");

  //  boolean boolY = gamepad2.y; // just to check if the code works
  //boolean boolX = gamepad2.x;
    //boolean boolA = gamepad2.a;
   // boolean boolB = gamepad2.b;

    // private CRServo slidesServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and servos
        RSlides = hardwareMap.get(DcMotorEx.class, "RSlides");
        LSlides = hardwareMap.get(DcMotorEx.class, "LSlides");
        //slidesServo = hardwareMap.get(CRServo.class, "slidesServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        bucketCServo = hardwareMap.get(CRServo.class, "bucketCServo");


        RSlides.setDirection(DcMotorEx.Direction.FORWARD);
        LSlides.setDirection(DcMotorEx.Direction.REVERSE);


        //slidesServo.setDirection(CRServo.Direction.FORWARD);


        RSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        RSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RSlides.setTargetPosition(0);
        LSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LSlides.setTargetPosition(0);
        LSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //slidesServo.setPower(0.0);



        waitForStart();


        while (opModeIsActive()) {

            this.slides(gamepad2.right_trigger);
            this.Bucket(-gamepad2.left_stick_y);
            this.ContinuousBucket(-gamepad2.right_stick_y);
            telemetry.addData("RSlides Power", RSlides.getPower());
            telemetry.addData("LSlides Power", LSlides.getPower());
            telemetry.addData("Lslides pos", LSlides.getCurrentPosition());
            telemetry.addData("lSlides target", LSlides.getTargetPosition());
            telemetry.addData("rslide pos", RSlides.getCurrentPosition());
            telemetry.addData("rslide target", RSlides.getTargetPosition());
            telemetry.addData("output", output);
            telemetry.addData("bucketServo position", x);
            telemetry.addData("bucketCont power", x2);

            // telemetry.addData("slidesServo Power", slidesServo.getPower());
            // telemetry.addData("slidesServo Power", slidesServo.getPower());
            telemetry.update();
        }


    }


    // Control the CRServo with buttons (adjust based on your setup)
    public void slides(double RT) {

        Rposition = RSlides.getTargetPosition();
        Lposition = LSlides.getTargetPosition();
        // boolean up = (y > 0.3);
        //boolean down = (y < -0.3);


        if( RT > 0.2) {
            targetPos = 1500;
            output = PIDControl1(targetPos, Rposition);
            RSlides.setPower(output * RT);
            LSlides.setPower(output * RT);
            LSlides.setTargetPosition(targetPos);
            RSlides.setTargetPosition(targetPos);
        }
        else if ((LSlides.getCurrentPosition()< 10 && LSlides.getCurrentPosition() > -10))
        {
            RSlides.setPower(0);
            LSlides.setPower(0);
        }

        else {
            targetPos = 0;
            output = PIDControl1(targetPos, Rposition);
            RSlides.setPower(output* 0.5);
            LSlides.setPower(output * 0.5);
            LSlides.setTargetPosition(targetPos);
            RSlides.setTargetPosition(targetPos);

        }


    }

    public double PIDControl1(double target, double state) {

        previousError1 = error1;
        error1 = target - state;
        if(error1 < 500 && error1 > -500) {

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

        }
        else if(error1 > 500) {
            output = 1.0;
        }
        else
        {
            output = -1.0;
        }
        return output;

    }

    public void Bucket(double ry)
    {
        double position = 120;
        if (ry>0.3 || ry < -0.3){
            bucketServo.setPosition(position);
        }  else {
           bucketServo.setPosition(0);
        }
        x= bucketServo.getPosition() * position;


    }
    public void ContinuousBucket(double ry)
    {
        if (ry>0.3 || ry < -0.3){
            bucketCServo.setPower(0.8 * ry);
        }  else {
            bucketCServo.setPower(0);
        }
        x2= bucketCServo.getPower();

    }
}

