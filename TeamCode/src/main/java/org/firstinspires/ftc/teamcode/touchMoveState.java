package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.ArrayList;

public class touchMoveState implements State{
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DigitalChannel ts;
    double targetDistance;
    State NextState;

    String turn = "null";
    boolean isMoved = false;

    public touchMoveState(ArrayList<DcMotor> motor, DigitalChannel touchSensor){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        ts = touchSensor;


    }
    public void setNextState(State state) {
        NextState  = state;

    }
    public void start(){

    }

    public String getTurn() {
        return turn;
    }

    public State update(){
        if(ts.getState() == true && !isMoved) {
            move("backward", .5);
            return this;
        } else if (ts.getState() == false && !isMoved) {
            isMoved = true;
            stop(leftFront, leftBack, rightFront, rightBack);
            return this;
        } else if (isMoved) {
            return NextState;
        } else{
            return this;
        }

    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }

    public void move(String direction, double speed) {
        switch (direction) {
            case "forward":
                //robot moves backwards
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                break;
            case "backward":
                //robot moves forwards
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                break;
            case "right":
                //robot strafes right
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
                break;
            case "left":
                //robot strafes left
                leftFront.setPower(speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(-speed);
                break;
            case "ccw":
                //robot turns clockwise(to the right)
                leftFront.setPower(speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(-speed);
                break;
            case "cw":
                //robot turns counterclockwise(to the left)
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
                break;
        }
    }


}
