package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleopVelocityTest extends LinearOpMode {

    // here: variables for ring motors, ring feeder servo, and motor power for tower goal and powershot
    DcMotor ringShooterMotor1, ringShooterMotor2;
    Servo ringFeederServo;
    double towerGoalMotorPower = -0.79, powerShotMotorPower = -0.65;

    @Override
    public void runOpMode() {

        // here: initialize ring motors, initialize ring feeder servo, and set initial position for ring feeder servo
        ringShooterMotor1 = hardwareMap.get(DcMotor.class, "ringShooterMotor1");
        ringShooterMotor2 = hardwareMap.get(DcMotor.class, "ringShooterMotor2");
        ringFeederServo = hardwareMap.get(Servo.class, "ringFeederServo");
        ringFeederServo.setPosition(0.6); // sets initial position of servo

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // here: add telemetry to see motor velocity


            // Press right trigger for ring feeder servo
            if (gamepad2.right_trigger == 1) {
                if (ringShooterMotor1.getPower() < -0.5){
                    ringFeederServo.setPosition(1);
                    sleep(300);
                    ringFeederServo.setPosition(0.6);
                }
            }

            // press Y for shooter motor for tower goal
            if (gamepad2.y) {
                if (ringShooterMotor1.getPower() < powerShotMotorPower) {
                    ringShooterMotor1.setPower(0);
                    ringShooterMotor2.setPower(0);
                }
                else {
                    ringShooterMotor1.setPower(towerGoalMotorPower);
                    ringShooterMotor2.setPower(towerGoalMotorPower);
                }
                sleep(1000);
            }

            // press X for shooter motor for powershot
            if (gamepad2.x) {
                ringShooterMotor1.setPower(powerShotMotorPower);
                ringShooterMotor2.setPower(powerShotMotorPower);
            }
        }
    }
}