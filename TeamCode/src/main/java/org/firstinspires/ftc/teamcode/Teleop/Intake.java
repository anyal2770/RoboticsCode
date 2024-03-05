package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


    //@TeleOp
    public class Intake  {
        //@Override
        private DcMotorEx Intake;
        public Intake (HardwareMap hardwareMap) {
            DcMotorEx Intake = hardwareMap.get(DcMotorEx.class, "Intake");
            Intake.setDirection(DcMotorEx.Direction.FORWARD);

            //waitForStart();
        }
           public void intakeMethod(double gamepadRT, boolean gamepadRB) {
               double x;
                if (gamepadRT>0.3){
                    Intake.setPower(1.0);
                } else if (gamepadRB == true) {
                    Intake.setPower(0.5);
                } else {
                    Intake.setPower(0);
                }
                x= Intake.getPower();
            }

           // telemetry.addData("power", x);
               // telemetry.update();

            }



