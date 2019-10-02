package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.ArrayList;


@Autonomous(name="FULL AUTO BLUTRAY", group="Iterative Opmode")
public class FULL_AUTO_BT extends OpMode
{

    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    /*
    ---SERVOS---
     */
    Servo armServo;


    /*
    ---SENSORS---
     */
    ModernRoboticsI2cRangeSensor sideSensor1;
    ModernRoboticsI2cRangeSensor sideSensor2; //Sense distance from wall when moving the tray
    DigitalChannel ts;

    /*
    ---STATES---
     */
    distanceMoveState rangeState;
    touchMoveState touchState;
    armMoveState armState;
    distanceMoveState rangeState2;




    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<ModernRoboticsI2cRangeSensor> mrrs = new ArrayList<ModernRoboticsI2cRangeSensor>();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = .75;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    int counter = 0;




    @Override
    public void init() {
        /*
        ---HARDWARE MAP---
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        armServo = hardwareMap.servo.get("arm motor");


        /*
        ---MOTOR DIRECTIONS---
         */
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        /*
        ---GROUPING---
         */
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        mrrs.add(sideSensor1);
        mrrs.add(sideSensor2);


        // Set all motors to zero power
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        /*
        ---USING STATES---
         */
        rangeState = new distanceMoveState(motors, sideSensor1, 16); //16 is a test value for now
        touchState = new touchMoveState(motors, ts);
        armState = new armMoveState(armServo, -1);
        rangeState2 = new distanceMoveState(motors, sideSensor2, 3);


        /*
        ---ORDERING STATES---
         */
        rangeState.setNextState(touchState);
        touchState.setNextState(armState);
        armState.setNextState(rangeState2);
        rangeState2.setNextState(null);
    }


    @Override
    public void start(){
        machine = new StateMachine(rangeState);

    }


    @Override
    public void loop()  {

        telemetry.addData("sensor 1", sideSensor1.getDistance(DistanceUnit.INCH));
        telemetry.addData("sensor 2", sideSensor2.getDistance(DistanceUnit.INCH));
        telemetry.addData("turn", rangeState.getTurn());

        telemetry.update();
        machine.update();

    }

    private StateMachine machine;

    @Override
    public void stop() {
    }

}


