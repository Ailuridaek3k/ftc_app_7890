package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;

/*
7890 Space Lions 2019 "FULL AUTO BLUTRAY"
author: 7890 Software
GOALS: Move the foundation, navigate under the bridge
DESCRIPTION: This code is used for our autonomous when we are located on the side of with the foundation tray.
 */
@Autonomous(name="FULL AUTO BLUTRAY HARD v2", group="Iterative Opmode")
public class BT_HARDCODE extends OpMode
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
    //there are no sensors in this code

    /*
    ---SENSORS---
     */
    DistanceSensor distanceSensor;
    //ModernRoboticsI2cRangeSensor distanceSensor;
    DigitalChannel ts;
    BNO055IMU imu;
    ColorSensor colorSensor;

    /*
    ---STATES---
     */
    MoveState rangeState;
    GyroTurnCWByPID turnState;
    touchMoveState touchState;
    MoveState rangeState2;
    armMotorState lockState;
    armMotorState lockState2;
    GyroTurnCCWByPID turnState2;
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

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
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
        //mrrs.add(distanceSensor);

        /*
        ---USING STATES---
         */

        //Moves the robot towards the wall near the tray until the we are 16 inches away.
        //Detects the distance from the wall using a range sensor.
        rangeState = new MoveState(motors, 2000, 0.5);

        //Turns the robot around 270 degrees clockwise (which is 90 degrees ccw) so that our
        //touch sensor is facing the foundation.
        turnState = new GyroTurnCWByPID(250, .3, motors, imu);

        //Drives until the touch sensor button is pressed by driving up against the foundation.
        //The purpose of this is to drive up and position ourself next to the tray so we can pull it.
        touchState = new touchMoveState(motors, ts);

        //Deploys the arm motor and attaches the robot to the tray so that we can pull it back
        //towards the building site.
        lockState = new armMotorState(armMotor, -0.7);

        //Moves our robot until we are close to the wall near the building site. Using our
        //range sensor we can detect our distance from the wall in inches and drag the tray
        //with us to score points in the building site.
        rangeState2 = new MoveState(motors, 3000, 0.5);

        //Detaches the robot from the tray so that we can leave it in the building site.
        //Moves the armMotor upwards.
        lockState2 = new armMotorState(armMotor, 0.0);

        //Drives up towards the bridge and stops once we are directly under it. Our color
        //sensor detects the colored tape on the ground and turns off the power in the wheels.
        parkState = new ColorSenseStopState(motors, colorSensor, "blue", 0.5, "left");

        /*
        ---ORDERING STATES---
         */

        rangeState.setNextState(turnState);
        turnState.setNextState(touchState);
        touchState.setNextState(lockState);
        lockState.setNextState(rangeState2);
        rangeState2.setNextState(lockState2);
        lockState2.setNextState(parkState);
        parkState.setNextState(null);
    }


    @Override
    public void start(){
        armMotor.setPower(0.0);

        leftFront.setPower(-0.3);
        leftBack.setPower(0.3);
        rightFront.setPower(0.3);
        rightBack.setPower(-0.3);
        wait(2);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        machine = new StateMachine(rangeState);
    }


    private StateMachine machine;
    public void loop()  {
        //telemetry.addData("angle: ", turnState2.getAngle());
        telemetry.update();

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


