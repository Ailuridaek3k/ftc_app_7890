package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;





public class gyroTurnState implements StateMachine.State {

    double targetAngle;

    double angle;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    BNO055IMU imu;




    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static double driveSpeed = 0.6;
    static final double TURN_SPEED = 0.5;


    private State NextState;

    public gyroTurnState(double angleTarget, double speed, ArrayList<DcMotor> motor, BNO055IMU IMU){

        driveSpeed = speed;
        targetAngle = angleTarget;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        imu = IMU;

    }

    public void setNextState(State state) {
        NextState  = state;

    }


    public void setAngle(double currentAngle) {
                angle = currentAngle;

    }
    @Override
    public void start() {
//         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//                parameters.mode                = BNO055IMU.SensorMode.IMU;//Reset the encoders back to zero for the next movement
//                parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                parameters.loggingEnabled      = false;rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//                  imu.initialize(parameters);
       rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        rightBack.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //Bring them back to using encoders
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //imu.initialize()

        //Setting their target to their current encoder value (should be zero) to the amount of inches times the counts per inches

    }

    @Override
    public StateMachine.State update() {


              //


             angle = imu.getAngularOrientation().firstAngle;




             if(angle < targetAngle){
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(driveSpeed);
                rightFront.setPower(driveSpeed);
                return this;
            }







            else {
                 leftBack.setPower(0);
                 leftFront.setPower(0);
                 rightBack.setPower(0);
                 rightFront.setPower(0);
                 return NextState;
             }

    }


}
