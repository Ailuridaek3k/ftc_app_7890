//Imports:
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


7890 Space Lions 2019 "FULL TELEOP"
author: 7890 Software
@TeleOp(name="FULL TELEOP", group="Tele Op")
public class FULL_TELEOP extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor liftMotor;

    /*
    ---DIRECTION SETUP---
     */
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    /*
    ---SERVOS---
     */
    Servo outtake;


    @Override
    public void init() {

        /*
        ---HARDWARE MAP---
        */
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        outtake = hardwareMap.servo.get("outtake");

        /*
        ---DIRECTIONS---
         */
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);

        outtake.setPosition(1.0);
    }

    @Override
    public void loop() {

        /*
        ---DRIVETRAIN---
         */
        float drive;
        float turn;
        float strafe;

        /*
        ---DRIVE CONTROLLER SETUP---
         */
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        /*
        ---MECANUM DRIVING CALCULATIONS---
         */
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        /*
        ---WHEEL POWERS---
         */
        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);

        /*
        ---LIFT CONTROLLER SETUP---
         */
        float liftControlUp = gamepad1.right_trigger;
        float liftControlDown = gamepad1.left_trigger;
        liftMotor.setPower(-liftControlDown);
        liftMotor.setPower(liftControlUp);

        /*
        ---TELEMETRY & TESTING---
         */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
