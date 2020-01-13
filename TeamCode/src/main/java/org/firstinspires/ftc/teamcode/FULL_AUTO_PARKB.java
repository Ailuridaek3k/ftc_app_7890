package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.ArrayList;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
7890 Space Lions 2019 "FULL AUTO PARKBLU"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
DESCRIPTION: This code is used for our autonomous when we are located on the side of the tray
 */
@Autonomous(name="FULL AUTO PARKBLU", group="Iterative Opmode")
public class FULL_AUTO_PARKB extends OpMode
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
    //ModernRoboticsI2cRangeSensor distanceSensor;
    DigitalChannel ts;
    BNO055IMU imu;
    ColorSensor colorSensor;

    /*
    ---STATES---
     */
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
        //armServo = hardwareMap.servo.get("arm motor");
        armMotor = hardwareMap.dcMotor.get("arm motor");

        //distanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distance sensor");
        ts = hardwareMap.get(DigitalChannel.class, "ts");
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
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
        //mrrs.add(distanceSensor);

        /*
        ---USING STATES---
         */
        parkState = new ColorSenseStopState(motors, colorSensor, "blue", 0.5, "forward");


        /*
        ---ORDERING STATES---
         */
        parkState.setNextState(null);

    }


    @Override
    public void start(){
        armMotor.setPower(0.0);
        machine = new StateMachine(parkState);
        //machine = new StateMachine(turnState);
    }


    private StateMachine machine;
    public void loop()  {

        //telemetry.addData("range sensor", distanceSensor.getDistance(DistanceUnit.INCH));
        //telemetry.addData("turn", rangeState.getTurn());
        //telemetry.addData("turn", turnState.getAngle());

        //telemetry.update();
        machine.update();

    }


//    @Override
//    public void stop() {
//    }

}


