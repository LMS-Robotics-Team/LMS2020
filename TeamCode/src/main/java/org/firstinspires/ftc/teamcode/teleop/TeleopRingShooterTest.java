package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleopRingShooterTest extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor ringShooterMotor1, ringShooterMotor2;

    @Override
    public void runOpMode() {

        // initialization
        ringShooterMotor1 = hardwareMap.get(DcMotor.class, "ringShooterMotor1");
        ringShooterMotor2 = hardwareMap.get(DcMotor.class, "ringShooterMotor2");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code for taking in rings
            if (gamepad2.b) {
                if (ringShooterMotor1.getPower() == 0){
                    ringShooterMotor1.setPower(0.5);
                    ringShooterMotor2.setPower(0.5);
                }
                else {
                    ringShooterMotor1.setPower(0);
                    ringShooterMotor2.setPower(0);
                }
            }

            telemetry.addData("Press B to start/stop ring shooter motors","");
            telemetry.update();
        }
    }
}