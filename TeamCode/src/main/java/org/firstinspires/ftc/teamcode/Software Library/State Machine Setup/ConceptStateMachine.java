package org.firstinspires.ftc.teamcode;
//This is a test for state machines, iterative opmode

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import java.util.Locale;


@Autonomous(name="State Machine Test1.2", group="Iterative Opmode")
public class ConceptStateMachine extends OpMode
{

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    ModernRoboticsI2cRangeSensor sideSensor1, sideSensor2, distSensor;
    rangeCalibrationState rangeState;
    distanceMoveState distanceState;
    timeState ts;



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
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        sideSensor1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ss1");
        sideSensor2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ss2");
        distSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ds");
        mrrs.add(sideSensor1);
        mrrs.add(sideSensor2);
        mrrs.add(distSensor);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);


        // Set all motors to zero power
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);


     //   distanceState = new distanceMoveState(motors, mrrs, 6.0);
        rangeState = new rangeCalibrationState(motors, mrrs, 3.0);
        ts = new timeState(0,0,motors, "forward");

       // distanceState.setNextState(rangeState);
        rangeState.setNextState(ts);
        ts.setNextState(null);



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
