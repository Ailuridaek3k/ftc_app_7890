package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.concurrent.TimeUnit;


import java.util.ArrayList;
/*
7890 Space Lions 2019 "ARM MOVE STATE"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
DESCRIPTION: This code is used to simplify our autonomous code for our arm.
 */
public class armMotorState implements State {
    DcMotor armMotor;
    double armPower;
    State NextState;

    public armMotorState(DcMotor lock, double power){
        armMotor = lock;
        armPower = power;
    }

    public void setNextState(State state) {
        NextState  = state;

    }
    public void start(){
        armMotor.setPower(0.0);
    }

    @Override
    public State update() {
        armMotor.setPower(armPower);
        wait(3);
        //TimeUnit.SECONDS.sleep(1);
        return NextState;

    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 1000);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
