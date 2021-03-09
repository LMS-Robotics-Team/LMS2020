package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleopWobbleGoalReleaseServoTest extends LinearOpMode {

    // sets variables for motors and servos
    Servo wobbleGoalReleaseServo;

    @Override
    public void runOpMode() {

        // initialization
        wobbleGoalReleaseServo = hardwareMap.get(Servo.class, "wobbleGoalReleaseServo");
        wobbleGoalReleaseServo.setPosition(0.3);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Press left trigger to unlock and lock wobble goal release servo
            if (gamepad2.left_trigger == 1) {
                if (wobbleGoalReleaseServo.getPosition() == 0) {
                    wobbleGoalReleaseServo.setPosition(0.3);
                }
                else {
                    wobbleGoalReleaseServo.setPosition(0);
                }
            }

            telemetry.addData("Hold left trigger to open/close wobble goal release servo","");
            telemetry.update();

        }
    }
}