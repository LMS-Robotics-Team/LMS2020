package org.firstinspires.ftc.teamcode.tests.oldtests;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Disabled
@Autonomous(name = "Test: Color Sound Test", group = "Tests")
public class ColorSoundTest extends LinearOpMode {
  // Define a variable for our color sensor
  ColorSensor color;

  // Declare OpMode members.
  private boolean goldFound;      // Sound file present flags
  private boolean shootFound;

  @Override
  public void runOpMode() {
    // Determine Resource IDs for sounds built into the RC application.
    int goldSoundID   = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());
    int shootSoundID   = hardwareMap.appContext.getResources().getIdentifier("shoot",   "raw", hardwareMap.appContext.getPackageName());

    // Determine if sound resources are found.
    // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
    if (goldSoundID != 0)
      goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

    if (shootSoundID != 0)
      shootFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, shootSoundID);

    // Get the color sensor from hardwareMap
    color = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Display sound status
    telemetry.addData("gold resource",   goldFound ?   "Found" : "NOT found\n Add gold.wav to /src/main/res/raw" );
    telemetry.addData("shoot resource", shootFound ? "Found" : "Not found\n Add shoot.wav to /src/main/res/raw" );

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData(">", "Press Start to continue");
    telemetry.update();

    // Wait for the Play button to be pressed
    waitForStart();

    // While the Op Mode is running, update the telemetry values.
    while (opModeIsActive()) {
      // say Gold each time gamepad B is pressed  (This sound is a resource)
      if (goldFound && (color.blue() > 2000) && (color.alpha() < 2000)) {
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
        telemetry.addData("Playing", "Resource Gold");
        telemetry.update();
      }

      // say Gold each time gamepad B is pressed  (This sound is a resource)
      if (shootFound && (color.blue() > 2000) && (color.alpha() > 2000)) {
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, shootSoundID);
        telemetry.addData("Playing", "Resource Shoot");
        telemetry.update();
      }


      telemetry.addData("Red", color.red());
      telemetry.addData("Green", color.green());
      telemetry.addData("Blue", color.blue());
      telemetry.addData("Alpha", color.alpha());
      telemetry.addData("Hue", color.argb());

      telemetry.update();
    }
  }
}
