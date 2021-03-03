package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class TeleopWobbleGoalServoTest extends LinearOpMode {

    // sets variables for motors and servos
    Servo wobbleGoalServo;

    @Override
    public void runOpMode() {

        // initialization
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");
        wobbleGoalServo.scaleRange(0.22,0.45);
        wobbleGoalServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code for rotating wobble goal grabber
            if (gamepad2.x){
                if (wobbleGoalServo.getPosition() != 0){
                    wobbleGoalServo.setPosition(0);
                }
                else {
                    wobbleGoalServo.setPosition(1);
                }
            }

            telemetry.addData("Press X to move wobble goal servo","");
            telemetry.update();

        }
    }
}