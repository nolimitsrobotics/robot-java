
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link MySensorMRRangeSensor} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "range sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "Sensor: MR range sensor", group = "Sensor")
  // comment out or remove this line to enable this opmode
public class MySensorMRRangeSensor extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override public void runOpMode() throws InterruptedException {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
