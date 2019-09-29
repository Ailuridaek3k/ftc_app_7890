package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.ArrayList;

public class armMoveState implements State {
    Servo armServo;
    double armPosition = -1;

    public armMoveState(Servo lock, double pos){
        armServo = lock;
        armPosition = pos;
    }

    public void setNextState(State state) {
        NextState  = state;

    }
    public void start(){

    }


    public State update() {
            armServo.setPosition(armPosition);


    }

    public void stop(Servo armServo) {
        //armServo.setPower(0.0);


    }

    public void move(String direction, double speed) {
        switch (direction) {
            case "forward":
                //arm moves forward
                armServo.setPower(speed);
                break;
            case "backward":
                //robot moves forwards
                armServo.setPower(-speed)
                break;

        }
    }


}
