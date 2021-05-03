package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop5 extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor driveFL, driveFR, driveBL, driveBR, takingInRingsMotor;
    private DcMotorEx ringShooterMotor1, ringShooterMotor2;
    Servo wobbleGoalServo, ringFeederServo, wobbleGoalReleaseServo;
    double drivePower = 0.8;
    double gearRatio = (17.0 / 12.0); // 1.4189;
    double wheelDiameter = 3.77953;
    double encoderTicksPerRotation = 537.6;
    double wheelCircumference = wheelDiameter * Math.PI;
    double encoderTicksPerInch = ((encoderTicksPerRotation / wheelCircumference) * gearRatio);
    double towerGoalMotorVelocity = -1640, powerShotMotorVelocity = -1280;
    double wobbleGoalVerticalPosition = 0.22, wobbleGoalReleasePosition = 0.48;
    double wobbleGoalLockClosed = 0, wobbleGoalLockOpen = 0.3;

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
            leftRight = gamepad1.right_trigger - gamepad1.left_trigger;
            rotate = gamepad1.right_stick_x;

            // driving in all directions and rotating
            driveFL.setPower(drivePower * (forwardBackward + leftRight + rotate));
            driveFR.setPower(drivePower * forwardBackward - leftRight - rotate);
            driveBL.setPower(drivePower * forwardBackward - leftRight + rotate);
            driveBR.setPower(drivePower * forwardBackward + leftRight - rotate);

            // function to update telemetry
            addTelemetry();

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
            }

            // Press right trigger for ring feeder servo
            if (gamepad2.right_trigger == 1) {
                if (ringShooterMotor1.getVelocity() < (powerShotMotorVelocity + 20)){
                    ringFeederServo.setPosition(1);
                    sleep(300);
                    ringFeederServo.setPosition(0.6);
                }
            }

            // Press left trigger to unlock and lock wobble goal release servo
            if (gamepad2.left_trigger == 1) {
                if (wobbleGoalReleaseServo.getPosition() < wobbleGoalLockOpen) {
                    wobbleGoalReleaseServo.setPosition(wobbleGoalLockOpen);
                }
                else {
                    wobbleGoalReleaseServo.setPosition(wobbleGoalLockClosed);
                }
            }

            // press Y for shooter motor for tower goal
            if (gamepad2.y) {
                if (ringShooterMotor1.getVelocity() < powerShotMotorVelocity) {
                    ringShooterMotor1.setPower(0);
                    ringShooterMotor2.setPower(0);
                }
                else {
                    ringShooterMotor1.setVelocity(towerGoalMotorVelocity);
                    ringShooterMotor2.setVelocity(towerGoalMotorVelocity);
                }
                sleep(1000);
            }

            // press X for shooter motor for powershot
            if (gamepad2.x) {
                ringShooterMotor1.setVelocity(powerShotMotorVelocity);
                ringShooterMotor2.setVelocity(powerShotMotorVelocity);
                sleep(1000);
            }

            // Press A to automatically shoot powershot in endgame
            if (gamepad1.a) {
                ringShooterMotor1.setVelocity(powerShotMotorVelocity);
                ringShooterMotor1.setVelocity(powerShotMotorVelocity);
                wobbleGoalServo.setPosition(wobbleGoalVerticalPosition);

                //move
                strafeRight(17);

                while (ringShooterMotor1.getVelocity() > (powerShotMotorVelocity + 20)) {
                    // do nothing
                }
                //shoot first powershot
                ringFeederServo.setPosition(1);
                sleep(2000);
                ringFeederServo.setPosition(0.6);

                //move
                strafeRight(7);

                while (ringShooterMotor1.getVelocity() > (powerShotMotorVelocity + 20)) {
                    // do nothing
                }
                //shoot second powershot
                ringFeederServo.setPosition(1);
                sleep(2000);
                ringFeederServo.setPosition(0.6);

                //move
                strafeRight(7);

                while (ringShooterMotor1.getVelocity() > (powerShotMotorVelocity + 20)) {
                    // do nothing
                }
                //shoot third powershot
                ringFeederServo.setPosition(1);
                sleep(2000);
                ringFeederServo.setPosition(0.6);
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
        ringShooterMotor1 = hardwareMap.get(DcMotorEx.class, "ringShooterMotor1");
        ringShooterMotor2 = hardwareMap.get(DcMotorEx.class, "ringShooterMotor2");

        // maps ring feeder servo variable to hardware configuration name
        ringFeederServo = hardwareMap.get(Servo.class, "ringFeederServo");
        ringFeederServo.setPosition(0.6); // sets initial position of servo

        // maps wobble goal servo variable to hardware configuration name
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");
        wobbleGoalServo.setPosition(wobbleGoalVerticalPosition); // sets initial position of servo

        wobbleGoalReleaseServo = hardwareMap.get(Servo.class, "wobbleGoalReleaseServo");
        wobbleGoalReleaseServo.setPosition(wobbleGoalLockOpen);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // class to add and update telemetry
    private void addTelemetry() {
        telemetry.addData("Ring shooter motor speed", ringShooterMotor1.getVelocity());
        telemetry.update();
    }

    private void strafeRight(int inches) {
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFL.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveFR.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveBL.setTargetPosition(-(int)(inches * encoderTicksPerInch));
        driveBR.setTargetPosition((int)(inches * encoderTicksPerInch));

        driveFL.setPower(drivePower);
        driveFR.setPower(-drivePower);
        driveBL.setPower(-drivePower);
        driveBR.setPower(drivePower);

        while (driveFL.isBusy()) {
            idle();
        }

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}