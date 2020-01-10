package org.firstinspires.ftc.teamcode;

/**
 * Created by wenhuizhao on 2/13/18.
 */
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "Rev distance Test", group = "LinearOpMode")
public class RevColorDistanceTest extends LinearOpMode {

    DistanceSensor depotSensor;

    @Override public void runOpMode() {

        // get a reference to our compass
        depotSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {


//            telemetry.addData("raw ultrasonic", depotSensor.rawUltrasonic());
//            telemetry.addData("raw optical", depotSensor.rawOptical());
//            telemetry.addData("cm optical", "%.2f cm", depotSensor.cmOptical());
            telemetry.addData("in", "%.2f in", depotSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}