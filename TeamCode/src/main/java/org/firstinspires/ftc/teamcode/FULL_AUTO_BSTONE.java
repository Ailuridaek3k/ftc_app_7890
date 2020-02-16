package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.ArrayList;

/*
7890 Space Lions 2019 "FULL AUTO PARKBLU"
author: 7890 Software (TEAM MEMBERS)
DESCRIPTION: This code is used for our autonomous when we are located on the side of the tray
 */
@Autonomous(name="FULL AUTO BSTONE", group="Iterative Opmode")
public class FULL_AUTO_BSTONE extends OpMode
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
    ---SENSORS---
     */
    DigitalChannel ts;
    BNO055IMU imu;
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor distanceSensor;
    ColorSensor stoneSensorL;
    ColorSensor stoneSensorR;
    
    /*
    ---STATES---
     */
    ColorSenseStopState initialMoveState;
    doubleColorState stoneState;
    armMotorState armState;
    distanceMoveState moveState;
    GyroTurnCCWByPID turnState;
    ColorSenseStopState parkState;
    MoveState moveState2;
    MoveState stopState;
    armMotorState releaseState;
    ColorSenseStopState parkState2;





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
        stoneSensorL = hardwareMap.get(ColorSensor.class, "stone sensor L");
        stoneSensorR = hardwareMap.get(ColorSensor.class, "stone sensor R");


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
        //Moves the robot backwards until the color sensors on the back sense a stone
        initialMoveState = new ColorSenseStopState(motors, stoneSensorL, "black and yellow", 0.3, "backward");
        
        //The robot strafes to the left until both color sensors on the back stop sensing yellow. 
        //This means we are at the Skystone.
        stoneState = new doubleColorState(motors, stoneSensorL, stoneSensorR, "black", 0.3, "left");
        
        //Deploys the arm motor and grabs the Skystone for delivery.
        armState = new armMotorState(armMotor, -0.3);
        
        //Moves the robot towards the wall, pulling the skystone, until we are 12 inches away from the wall.
        //Detects the distance from the wall using a range sensor
        moveState = new distanceMoveState(motors, distanceSensor, 12, 0.5);
        
        //Turns the robot around 90 degrees counterclockwise so that the back of the robot is facing the alliance bridge.
        turnState = new GyroTurnCCWByPID(70, 0.3, motors, imu);
        
        //Drives up towards the bridge until the color sensor on the bottom senses the blue tape.
        parkState = new ColorSenseStopState(motors, colorSensor, "blue", 0.5, "backward");
        
        //Moves backwards for 100 miliseconds to fully deliver the Skystone.
        moveState2 = new MoveState(motors, 100, -0.5);
        
        //Stops moving the wheels
        stopState = new MoveState(motors, 300, 0.0);
        
        //Releases the arm to complete the Skystone delivery
        releaseState = new armMotorState(armMotor, 0.3);
        
        //Drives up towards the bridge and stops once we are directly under it. Our color
        //sensor detects the colored tape on the ground and turns off the power in the wheels.
        parkState2 = new ColorSenseStopState(motors, colorSensor, "blue", 0.5, "forward");

        /*
        ---ORDERING STATES---
         */
        initialMoveState.setNextState(stoneState);
        stoneState.setNextState(armState);
        armState.setNextState(moveState);
        moveState.setNextState(turnState);
        turnState.setNextState(parkState);
        parkState.setNextState(moveState2);
        moveState2.setNextState(stopState);
        stopState.setNextState(releaseState);
        releaseState.setNextState(parkState2);
        parkState2.setNextState(null);

    }


    @Override
    public void start(){
        armMotor.setPower(0.0);
        
        //Start state machine
        machine = new StateMachine(initialMoveState);
    }


    private StateMachine machine;
    public void loop()  {
        //Running state machine
        machine.update();

    }
}


