package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.ArrayList;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

/*
7890 Space Lions 2019 "FULL AUTO REDTRAY"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
DESCRIPTION: This code is used for our autonomous when we are located on the red side of the tray
 */
@Autonomous(name="FULL AUTO REDTRAY", group="Iterative Opmode")
public class FULL_AUTO_RT extends OpMode
{

    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor armMotor;

    /*
    ---SERVOS---
     */
    //Servo armServo;


    /*
    ---SENSORS---
     */
    ModernRoboticsI2cRangeSensor distanceSensor;
    DigitalChannel ts;
    BNO055IMU imu;
    ColorSensor colorSensor;

    /*
    ---STATES---
     */
    distanceMoveState rangeState;
    GyroTurnCWByPID turnState;
    touchMoveState touchState;
    distanceMoveState rangeState2;
    armMotorState lockState;
    armMotorState lockState2;
    GyroTurnCWByPID turnState2;
    ColorSenseStopState parkState;



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

    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu"); // lmao hardware what a joke

        imu.initialize(parameters);

        /*
        ---HARDWARE MAP---
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        armMotor = hardwareMap.dcMotor.get("arm motor");


        distanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distance sensor");
        ts = hardwareMap.get(DigitalChannel.class, "ts");
        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");

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
        mrrs.add(distanceSensor);

        /*
        ---USING STATES---
         */
        rangeState = new distanceMoveState(motors, distanceSensor, 16, 0.5); //16 is a test value for now
        turnState = new GyroTurnCWByPID(80, .3, motors, imu);
        touchState = new touchMoveState(motors, ts);
        //armState = new armMoveState(armServo, 1.0);
        lockState = new armMotorState(armMotor, -0.5);
        rangeState2 = new distanceMoveState(motors, distanceSensor, 9, 0.3);
        lockState2 = new armMotorState(armMotor, 0.0);
        turnState2 = new GyroTurnCWByPID(80, .3, motors, imu);
        parkState = new ColorSenseStopState(motors, colorSensor, "red", 0.5, "forward");

        /*
        ---ORDERING STATES---
         */
        rangeState.setNextState(turnState);
        turnState.setNextState(touchState);
        touchState.setNextState(lockState);
        lockState.setNextState(rangeState2);
        rangeState2.setNextState(turnState2);
        turnState2.setNextState(lockState2);
        lockState2.setNextState(parkState);
        parkState.setNextState(null);
    }


    @Override
    public void start(){
        armMotor.setPower(0.0);

        leftFront.setPower(0.3);
        leftBack.setPower(-0.3);
        rightFront.setPower(-0.3);
        rightBack.setPower(0.3);
        wait(2);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        machine = new StateMachine(rangeState);
    }


    private StateMachine machine;
    public void loop()  {


        machine.update();
    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 1000);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
