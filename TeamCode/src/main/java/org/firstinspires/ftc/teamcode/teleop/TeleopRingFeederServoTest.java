package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleopRingFeederServoTest extends LinearOpMode {

    // sets variables for motors and servos
    Servo ringFeederServo;

    @Override
    public void runOpMode() {

        // initialization
        ringFeederServo = hardwareMap.get(Servo.class, "ringFeederServo");
        ringFeederServo.setPosition(0.6);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // feeder servo
            if (gamepad2.y){
                if (ringFeederServo.getPosition() != 0.9){
                    ringFeederServo.setPosition(0.9);
                    sleep(500);
                    ringFeederServo.setPosition(0.6);
                }
                else {
                    ringFeederServo.setPosition(0.6);
                }
            }

            telemetry.addData("Press Y to move ring feeder servo","");
            telemetry.update();

        }
    }
}