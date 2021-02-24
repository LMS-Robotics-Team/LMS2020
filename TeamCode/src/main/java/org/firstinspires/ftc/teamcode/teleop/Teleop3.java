package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class Teleop3 extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor driveFL, driveFR, driveBL, driveBR;
    Servo wobbleGoalServo, ringFeederServo;
    DcMotor takingInRingsMotor, ringShooterMotor;

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
            forwardBackward = -gamepad1.right_stick_y;
            leftRight = gamepad1.right_stick_x;
            rotate = gamepad1.right_trigger - gamepad1.left_trigger;

           // driving in all directions and rotating
            driveFL.setPower(forwardBackward + leftRight + rotate);
            driveFR.setPower(forwardBackward - leftRight - rotate);
            driveBL.setPower(forwardBackward - leftRight + rotate);
            driveBR.setPower(forwardBackward + leftRight - rotate);

            // code for taking in rings
            if (gamepad2.a) {
                if (takingInRingsMotor.getPower() == 0){
                    takingInRingsMotor.setPower(0.5);
                }
                else {
                    takingInRingsMotor.setPower(0);
                }
            }

            // code for rotating wobble goal grabber
            if (gamepad2.b){
                if (wobbleGoalServo.getPosition() != 0){
                    wobbleGoalServo.setPosition(0);
                }
                else {
                    wobbleGoalServo.setPosition(1);
                }
            }

            // feeder servo
            if (gamepad2.x){
                if (ringFeederServo.getPosition() == 0.6){
                    ringFeederServo.setPosition(0.9);
                    sleep(500);
                    ringFeederServo.setPosition(0.6);
                }
                else {
                    ringFeederServo.setPosition(0.6);
                }
            }


            // shooter motor
            if (gamepad2.y) {
                if (ringShooterMotor.getPower() == 0) {
                    ringShooterMotor.setPower(0.5);
                } else {
                    ringShooterMotor.setPower(0);
                }
            }

            // function to update telemetry
            addTelemetry();
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

        ringFeederServo = hardwareMap.get(Servo.class, "ringFeederServo");
//        ringFeederServo.scaleRange(0.6, 0.9);
//        ringFeederServo.setPosition(0.6);

/*        takingInRingsMotor = hardwareMap.get(DcMotor.class, "takingInRingsMotor");
        ringShooterMotor = hardwareMap.get(DcMotor.class, "ringShooterMotor");

        */

        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");
        wobbleGoalServo.scaleRange(0.22,0.45);
//        wobbleGoalServo.setPosition(0);

        // sets right motors to reverse direction so they're going the right way
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);

        // sets drive motor zero power behavior to brake
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // class to add and update telemetry
    private void addTelemetry() {
        telemetry.addData("gamepad1","");
        telemetry.addData("Right stick up/down"," Forward/backward");
        telemetry.addData("Right stick left/right"," Strafe left/right");
        telemetry.addData("Left and right triggers"," Rotate left/right");
        telemetry.update();
    }
}