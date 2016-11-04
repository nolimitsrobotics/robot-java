

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "ods sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR ODS", group = "Sensor")
@Disabled
public class MySensorMROpticalDistance extends LinearOpMode {

  OpticalDistanceSensor odsSensor;  // Hardware Device Object

  @Override
  public void runOpMode() throws InterruptedException {

    // get a reference to our Light Sensor object.
    odsSensor = hardwareMap.opticalDistanceSensor.get("ods");

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // send the info back to driver station using telemetry function.
      telemetry.addData("Raw",    odsSensor.getRawLightDetected());
      telemetry.addData("Normal", odsSensor.getLightDetected());

      telemetry.update();
      idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
  }
}
