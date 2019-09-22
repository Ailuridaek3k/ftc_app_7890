package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

public class rangeCalibrationState implements State{
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    double targetDistance;
    State NextState;
    ModernRoboticsI2cRangeSensor sideSensor1, sideSensor2, distSensor;
    String turn = "null";
    boolean isMoved = false;
    boolean hasAligned = false;

    public rangeCalibrationState( ArrayList<DcMotor> motor,  ArrayList<ModernRoboticsI2cRangeSensor> mrrs, double dist){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        sideSensor1 = mrrs.get(0);
        sideSensor2 = mrrs.get(1);
        distSensor = mrrs.get(2);
        targetDistance = dist;


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

            if ((Math.round((sideSensor1.getDistance(DistanceUnit.CM)*10))/10) == ((Math.round(sideSensor2.getDistance(DistanceUnit.CM)*10))/10) && !isMoved &&
                distSensor.getDistance(DistanceUnit.INCH) > 3.0){
            //still not at target distance
            turn = "moving";
            move("forward", .5);
            hasAligned = true;
            return this;
            }
            else if ((((sideSensor1.getDistance(DistanceUnit.INCH)))) > (((sideSensor2.getDistance(DistanceUnit.INCH)))) && !isMoved && !hasAligned){
                move("cw", .15);
                turn = "cw";
                return this;
            } else if ((((sideSensor1.getDistance(DistanceUnit.INCH)))) < (((sideSensor2.getDistance(DistanceUnit.INCH)))) && !isMoved && !hasAligned) {
                move("ccw", .15);
                turn = "ccw";
                return this;
            }
            //SECONDARY
            else if ((sideSensor1.getDistance(DistanceUnit.INCH)) > ((Math.round(sideSensor2.getDistance(DistanceUnit.INCH) + 1)))
                    || (((sideSensor1.getDistance(DistanceUnit.INCH) + 1))) < (((sideSensor2.getDistance(DistanceUnit.INCH))))
                    && !isMoved && hasAligned){
                turn = "unaligned!";
                hasAligned = false;
                return this;
            } else if (sideSensor1.getDistance(DistanceUnit.INCH) == sideSensor2.getDistance(DistanceUnit.INCH) && !isMoved &&
                    distSensor.getDistance(DistanceUnit.INCH) <= 3.0){
                //at target distance
                turn = "finished";
                isMoved = true;
                return this;
            } else if (isMoved){
                return NextState;
            } else{
                return this;
            }

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
