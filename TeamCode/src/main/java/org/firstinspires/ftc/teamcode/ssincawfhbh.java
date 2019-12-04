//Imports:
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
7890 Space Lions 2019 "FULL TELEOP"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
 */
@TeleOp(name="ssincawfhbh", group="Tele Op")
public class ssincawfhbh extends OpMode {
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
    Servo armServo;
    DigitalChannel ts;

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
        ts = hardwareMap.get(DigitalChannel.class, "ts");
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        intakeMotor = hardwareMap.dcMotor.get("intake motor");
      //  lockMotor = hardwareMap.dcMotor.get("lock motor");
        // lock = hardwareMap.servo.get("lock");

        armServo = hardwareMap.servo.get("arm motor");

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
        if(ts.getState() == true){
            telemetry.addLine("not press");
            telemetry.update();
        }else if(ts.getState() == false){
            telemetry.addLine("press");
            telemetry.update();
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
}
