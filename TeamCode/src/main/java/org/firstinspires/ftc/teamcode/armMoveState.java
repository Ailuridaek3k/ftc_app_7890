package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.ArrayList;
/*
7890 Space Lions 2019 "ARM MOVE STATE"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
DESCRIPTION: This code is used to simplify our autonomous code for our arm.
 */
public class armMoveState implements State {
    Servo armServo;
    double armPosition;
    State NextState;

    public armMoveState(Servo lock, double pos){
        armServo = lock;
        armPosition = pos;
    }

    public void setNextState(State state) {
        NextState  = state;

    }
    public void start(){
        armServo.setPosition(armPosition);
    }

    @Override
    public State update() {
        armServo.setPosition(armPosition);
        return NextState;

    }



}
