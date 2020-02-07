package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

public class doubleColorState implements State{
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    State NextState;
    ColorSensor cs1;
    ColorSensor cs2;
    String cval;
    String dir;
    double pow;
    String turn = "null";

    public doubleColorState(ArrayList<DcMotor> motor, ColorSensor colorSensor, ColorSensor colorSensor2, String color, double power, String direction){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        dir = direction;
        cs1 = colorSensor;
        cs2 = colorSensor2;
        pow = power;
        cval = color;


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

        if(cval.equals("yellow")){
            if(dir.equals("forward")){
                leftBack.setPower(-pow);
                leftFront.setPower(-pow);
                rightBack.setPower(-pow);
                rightFront.setPower(-pow);
            }
            else if(dir.equals("backward")){
                leftBack.setPower(pow);
                leftFront.setPower(pow);
                rightBack.setPower(pow);
                rightFront.setPower(pow);
            }
            else if (dir.equals("right")) {//robot strafes right
                leftFront.setPower(pow);
                rightFront.setPower(-pow);
                leftBack.setPower(-pow);
                rightBack.setPower(pow);
            }
            else if(dir.equals("left")) {
                leftFront.setPower(-pow);
                rightFront.setPower(pow);
                leftBack.setPower(pow);
                rightBack.setPower(-pow);
            }

            if(cs1.blue()<cs1.red() && cs1.blue()<cs1.green() && cs2.blue()<cs2.red() && cs2.blue()<cs2.green()){
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return NextState;
            }


            return this;
        }
        else if(cval.equals("black")){
            if(dir.equals("forward")){
                leftBack.setPower(-pow);
                leftFront.setPower(-pow);
                rightBack.setPower(-pow);
                rightFront.setPower(-pow);
            }
            else if(dir.equals("backward")){
                leftBack.setPower(pow);
                leftFront.setPower(pow);
                rightBack.setPower(pow);
                rightFront.setPower(pow);
            }
            else if (dir.equals("right")) {//robot strafes right
                leftFront.setPower(pow);
                rightFront.setPower(-pow);
                leftBack.setPower(-pow);
                rightBack.setPower(pow);
            }
            else if(dir.equals("left")) {
                leftFront.setPower(-pow);
                rightFront.setPower(pow);
                leftBack.setPower(pow);
                rightBack.setPower(-pow);
            }

            if(cs1.blue()<100 && cs1.green()<100 && cs1.red()<100 && cs2.blue()<100 && cs2.green()<100 && cs2.red()<100){
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return NextState;
            }


            return this;
        }
        return this;
    }

    public void move(String direction, double speed) {

        switch (direction) {
            case "forward":
                //robot moves forwards
                //if(hasAligned){
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);

                //hasAligned = false;}
                break;
            case "backward":
                //if(hasAligned){
                //robot moves backwards
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                //hasAligned = false;}
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
                break;}
        }



    public void wait(int time) {
        try {
            Thread.sleep(time * 100);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }

}
