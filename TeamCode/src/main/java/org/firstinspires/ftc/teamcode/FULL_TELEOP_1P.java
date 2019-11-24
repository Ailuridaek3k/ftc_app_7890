//Imports:
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
7890 Space Lions 2019 "FULL TELEOP"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
 */
@TeleOp(name="FULL TELEOP 1P", group="Tele Op")
public class FULL_TELEOP_1P extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor liftMotor;
    DcMotor intakeMotor;
    DcMotor lockMotor;
    Servo lock;

    /*
    ---DIRECTION SETUP---
     */
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    /*
    ---SERVOS---
     */
    //no servos for now

    int spin = 0;

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
        intakeMotor = hardwareMap.dcMotor.get("intake motor");
      //  lockMotor = hardwareMap.dcMotor.get("lock motor");
        // lock = hardwareMap.servo.get("lock");

        /*
        ---DIRECTIONS---
         */
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);

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


//        double lfDrive = 0;
//        double lbDrive = 0;
//        double rfDrive = 0;
//        double rbDrive = 0;
//
//        if(strafe > 0.1){
//            lfDrive = -1;
//            lbDrive = 1;
//            rfDrive = 1;
//            rbDrive = -1;
//        }else if(strafe < -0.1) {
//            lfDrive = 1;
//            lbDrive = -1;
//            rfDrive = -1;
//            rbDrive = 1;
//        }
//        else if(turn > 0.1){
//            lfDrive = 1;
//            lbDrive = 1;
//            rfDrive = -1;
//            rbDrive = -1;
//        }else if(turn < -0.1) {
//            lfDrive = -1;
//            lbDrive = -1;
//            rfDrive = 1;
//            rbDrive = 1;
//        }
//        else if(drive > 0.1){
//            lfDrive = 1;
//            lbDrive = -1;
//            rfDrive = 1;
//            rbDrive = -1;
//
//        }else if(drive < -0.1) {
//            lfDrive = -1;
//            lbDrive = 1;
//            rfDrive = -1;
//            rbDrive = 1;
//        }

        /*
        ---WHEEL POWERS---
         */
        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);

        //problem: left stick y strafes, left stick x turns, right stick x moves forward and back

        /*
        ---LIFT CONTROLLER SETUP---
         */
        float liftControlUp = gamepad1.right_trigger;
        float liftControlDown = gamepad1.left_trigger;
        liftMotor.setPower(-liftControlDown);
        liftMotor.setPower(liftControlUp);

        /*
        ---INTAKE WHEELS---
         */
        if(gamepad1.x && spin == 0){
            //first time around
            spin = 1; //whenever you turn it on it will do 1
        }else if(gamepad1.x && (spin != 1)){
            spin = 1;
        }else if(gamepad1.x && (spin != -1)){
            spin = -1;
        }else if(gamepad1.y){
            spin = 0;
        }
        intakeMotor.setPower(spin);

        /*
        ---LOCKING MECHANISM---
         */
//        double lockSpeed = 0.7; //a testing value for now
//        while(gamepad1.dpad_up) {
//            lockMotor.setPower(-lockSpeed);
//        }
//        while (gamepad1.dpad_down){
//            lockMotor.setPower(lockSpeed);
//        }
/*
        while(gamepad1.dpad_up) {
            lock.setPosition(-1.0);
        }
        while (gamepad1.dpad_down){
            lock.setPosition(1.0);
        }
*/
        /*
        ---TELEMETRY & TESTING---
         */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
