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
    DcMotor armMotor;

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
        armMotor = hardwareMap.dcMotor.get("arm motor");

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
        //We calculate the power needed to set to each wheel by using a variable for drive,
        //turn, and strafe. By adding and subtracting these three variables against each
        //other we can calculate the power needed for each wheel in every situation.

        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        /*
        ---WHEEL POWERS---
         */
        //We set the power to wheels based off the values calculated above. We can move
        //in all directions based off of the combinations inputted by the driver.

        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);

        /*
        ---LIFT CONTROLLER SETUP---
         */
        //The lift controls are on the trigger so that the driver can both move and
        //raise + lower the lift at the same time. This allows for a more efficient tele-op.

        float liftControlUp = gamepad1.right_trigger;
        float liftControlDown = gamepad1.left_trigger;
        liftMotor.setPower(-liftControlDown);
        liftMotor.setPower(liftControlUp);

        /*
        ---INTAKE WHEELS---
         */
        //These intake wheels are what we use to suck the stones up into our intake
        //mechanism. You press the button once to activate the wheels and then press
        //the button again to switch the direction of the wheels.

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

        if (spin == 1){
            telemetry.addLine("OUT");
            telemetry.update();
        }else if(spin == -1){
            telemetry.addLine("IN");
            telemetry.update();
        }else{

        }


        /*
        ---CONTROLLING THE ARM---
         */
        //This code gives us manual control the arm that we use to attach ourselves
        //to the foundation. This means that we are able to reposition the tray during
        //tele-op if needed.

        if(gamepad1.dpad_up) {
            armMotor.setPower(0.3);
        }
        if(gamepad1.dpad_down) {
            armMotor.setPower(-0.5);
        }

        /*
        ---TELEMETRY & TESTING---
         */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
