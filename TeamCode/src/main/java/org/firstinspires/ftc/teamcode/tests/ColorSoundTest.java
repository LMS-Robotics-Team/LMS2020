package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Test: Color Sensor", group = "Tests")
public class ColorSoundTest extends LinearOpMode {
  // Define a variable for our color sensor
  ColorSensor color;

  @Override
  public void runOpMode() {
    // Get the color sensor from hardwareMap
    color = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Wait for the Play button to be pressed
    waitForStart();

    // While the Op Mode is running, update the telemetry values.
    while (opModeIsActive()) {
      telemetry.addData("Red", color.red());
      telemetry.addData("Green", color.green());
      telemetry.addData("Blue", color.blue());
      telemetry.addData("Alpha", color.alpha());
      telemetry.addData("Hue", color.argb());

      telemetry.update();
    }
  }
}
