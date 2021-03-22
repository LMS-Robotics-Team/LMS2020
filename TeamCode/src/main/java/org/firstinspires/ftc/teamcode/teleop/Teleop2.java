package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Teleop2 extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor driveFL, driveFR, driveBL, driveBR, takingInRingsMotor, ringShooterMotor1, ringShooterMotor2;
    Servo wobbleGoalServo, ringFeederServo, wobbleGoalReleaseServo;
    Orientation angles;
    double towerGoalRingMotorSpeed = -0.79, powerShotRingMotorSpeed = -0.7;
    double wobbleGoalVerticalPosition = 0.22, wobbleGoalReleasePosition = 0.48;

    // creates variables for drive inputs from controllers
    private double forwardBackward, leftRight, rotate;

    @Override
    public void runOpMode() {

        // function to do init routine
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // sets values of variables for gamepad1 (start+a) inputs for driving
            forwardBackward = -gamepad1.left_stick_y;
            leftRight = gamepad1.right_stick_x;
            rotate = gamepad1.right_trigger - gamepad1.left_trigger;

            // function to update telemetry
            addTelemetry();

            // driving in all directions and rotating
            driveFL.setPower(forwardBackward + leftRight + rotate);
            driveFR.setPower(forwardBackward - leftRight - rotate);
            driveBL.setPower(forwardBackward - leftRight + rotate);
            driveBR.setPower(forwardBackward + leftRight - rotate);

            // Press right bumper to have ring intake take in rings
            if (gamepad2.right_bumper) {
                takingInRingsMotor.setPower(-1);
            }

            // Press left bumper to have ring intake run in reverse to spit out rings
            if (gamepad2.left_bumper) {
                takingInRingsMotor.setPower(1);
            }

            // Press A to shut off ring intake motor
            if (gamepad2.a) {
                takingInRingsMotor.setPower(0);
            }

            // Press B for wobble goal servo horizontal and vertical positions
            if (gamepad2.b) {
                if (wobbleGoalServo.getPosition() < 0.3) {
                    wobbleGoalServo.setPosition(wobbleGoalReleasePosition);
                    wobbleGoalReleaseServo.setPosition(0);
                }
                else {
                    wobbleGoalServo.setPosition(wobbleGoalVerticalPosition);
                }
                sleep(500);
            }

            // Press right trigger for ring feeder servo
            if (gamepad2.right_trigger == 1) {
                if (ringShooterMotor1.getPower() < -0.5){
                    ringFeederServo.setPosition(1);
                    sleep(300);
                    ringFeederServo.setPosition(0.6);
                }
            }

            // Press left trigger to unlock and lock wobble goal release servo
            if (gamepad2.left_trigger == 1) {
                if (wobbleGoalReleaseServo.getPosition() < 0.3) {
                    wobbleGoalReleaseServo.setPosition(0.3);
                }
                else {
                    wobbleGoalReleaseServo.setPosition(0);
                }
                sleep(1000);
            }

            // press Y for shooter motor for tower goal
            if (gamepad2.y) {
                if (ringShooterMotor1.getPower() < powerShotRingMotorSpeed) {
                    ringShooterMotor1.setPower(0);
                    ringShooterMotor2.setPower(0);
                }
                else {
                    ringShooterMotor1.setPower(towerGoalRingMotorSpeed);
                    ringShooterMotor2.setPower(towerGoalRingMotorSpeed);
                }
                sleep(1000);
            }

            // press X for shooter motor for powershot
            if (gamepad2.x) {
                ringShooterMotor1.setPower(powerShotRingMotorSpeed);
                ringShooterMotor2.setPower(powerShotRingMotorSpeed);
            }

            if (gamepad2.dpad_up) {
                ringShooterMotor1.setPower(-0.75);
                ringShooterMotor2.setPower(-0.75);
            }

            if (gamepad2.dpad_down) {
                ringShooterMotor1.setPower(-0.7);
                ringShooterMotor2.setPower(-0.7);
            }

            if (gamepad2.dpad_left) {
                ringShooterMotor1.setPower(-0.6);
                ringShooterMotor2.setPower(-0.6);
            }

            if (gamepad2.dpad_right) {
                ringShooterMotor1.setPower(-0.55);
                ringShooterMotor2.setPower(-0.55);
            }


        }
    }

    // initialization class
    private void initialize() {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // maps drive motor variables to hardware configuration names
        driveFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        driveFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        driveBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        driveBR = hardwareMap.get(DcMotor.class, "motorTestBR");

        // sets left motors to reverse direction so they're going the right way
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);

        // sets drive motor zero power behavior to brake
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // maps ring intake motor variable to hardware configuration name
        takingInRingsMotor = hardwareMap.get(DcMotor.class, "takingInRingsMotor");

        // maps ring shooter motor variables to hardware configuration names
        ringShooterMotor1 = hardwareMap.get(DcMotor.class, "ringShooterMotor1");
        ringShooterMotor2 = hardwareMap.get(DcMotor.class, "ringShooterMotor2");

        // maps ring feeder servo variable to hardware configuration name
        ringFeederServo = hardwareMap.get(Servo.class, "ringFeederServo");
        ringFeederServo.setPosition(0.6); // sets initial position of servo

        // maps wobble goal servo variable to hardware configuration name
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");
        wobbleGoalServo.setPosition(wobbleGoalVerticalPosition); // sets initial position of servo

        wobbleGoalReleaseServo = hardwareMap.get(Servo.class, "wobbleGoalReleaseServo");
        wobbleGoalReleaseServo.setPosition(0);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // class to add and update telemetry
    private void addTelemetry() {
        telemetry.addData("Ring shooter motor speed", ringShooterMotor1.getPower());
        telemetry.addData("Ring intake motor speed", takingInRingsMotor.getPower());

        telemetry.update();
    }
}