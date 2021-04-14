package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TeleopOneGamepad extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor driveFL, driveFR, driveBL, driveBR, takingInRingsMotor;
    private DcMotorEx ringShooterMotor1, ringShooterMotor2;
    Servo wobbleGoalServo, ringFeederServo, wobbleGoalReleaseServo;
    Orientation angles;
    double towerGoalRingMotorSpeed = -0.79, powerShotRingMotorSpeed = -0.65;
    double towerGoalMotorVelocity = -1660, powerShootMotorVelocity = -1350;
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
            leftRight = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            // driving in all directions and rotating
            driveFL.setPower(forwardBackward + leftRight + rotate);
            driveFR.setPower(forwardBackward - leftRight - rotate);
            driveBL.setPower(forwardBackward - leftRight + rotate);
            driveBR.setPower(forwardBackward + leftRight - rotate);

            // function to update telemetry
            addTelemetry();

            // Press right bumper to have ring intake take in rings
            if (gamepad1.right_bumper) {
                takingInRingsMotor.setPower(-1);
            }

            // Press left bumper to have ring intake run in reverse to spit out rings
            if (gamepad1.left_bumper) {
                takingInRingsMotor.setPower(1);
            }

            // Press A to shut off ring intake motor
            if (gamepad1.a) {
                takingInRingsMotor.setPower(0);
            }

            // Press B for wobble goal servo horizontal and vertical positions
            if (gamepad1.b) {
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
            if (gamepad1.right_trigger == 1) {
                if (ringShooterMotor1.getVelocity() <= powerShootMotorVelocity){
                    ringFeederServo.setPosition(1);
                    sleep(300);
                    ringFeederServo.setPosition(0.6);
                }
            }

            // press Y for shooter motor for tower goal
            if (gamepad1.y) {
                if (ringShooterMotor1.getVelocity() < powerShootMotorVelocity) {
                    ringShooterMotor1.setVelocity(0);
                    ringShooterMotor2.setVelocity(0);
                }
                else {
                    ringShooterMotor1.setVelocity(towerGoalMotorVelocity);
                    ringShooterMotor2.setVelocity(towerGoalMotorVelocity);
                }
                sleep(1000);
            }

            // press X for shooter motor for powershot
            if (gamepad1.x) {
                ringShooterMotor1.setVelocity(powerShootMotorVelocity);
                ringShooterMotor2.setVelocity(powerShootMotorVelocity);
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