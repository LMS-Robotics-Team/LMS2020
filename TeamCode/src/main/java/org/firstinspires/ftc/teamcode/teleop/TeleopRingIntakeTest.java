package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleopRingIntakeTest extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor takingInRingsMotor;

    @Override
    public void runOpMode() {

        // initialization
        takingInRingsMotor = hardwareMap.get(DcMotor.class, "takingInRingsMotor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code for taking in rings
            if (gamepad2.a) {
                if (takingInRingsMotor.getPower() == 0){
                    takingInRingsMotor.setPower(-1.0);
                }
                else {
                    takingInRingsMotor.setPower(0);
                }
            }

        telemetry.addData("Press A to start/stop ring intake motor","");
        telemetry.update();
        }
    }
}