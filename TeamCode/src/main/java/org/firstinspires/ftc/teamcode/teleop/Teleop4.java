package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop4 extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor driveFL, driveFR, driveBL, driveBR, takingInRingsMotor, ringShooterMotor1, ringShooterMotor2;
    Servo wobbleGoalServo, ringFeederServo;

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

            ringShooterMotor1.setPower(-0.9);
            ringShooterMotor2.setPower(-0.9);
            takingInRingsMotor.setPower(-1);

            // sets values of variables for gamepad1 (start+a) inputs for driving
            forwardBackward = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            leftRight = gamepad1.right_trigger - gamepad1.left_trigger;
            rotate = -gamepad1.left_stick_y + gamepad1.right_stick_y;

           // driving in all directions and rotating
            driveFL.setPower(forwardBackward + leftRight + rotate);
            driveFR.setPower(forwardBackward - leftRight - rotate);
            driveBL.setPower(forwardBackward - leftRight + rotate);
            driveBR.setPower(forwardBackward + leftRight - rotate);

            // code for taking in rings
            if (gamepad2.a) {
                if (takingInRingsMotor.getPower() == 0){
                    takingInRingsMotor.setPower(-1.0);
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
            if (gamepad2.right_trigger == 1) {
                if (ringShooterMotor1.getPower() < -0.8){
                    ringFeederServo.setPosition(0.9);
                    sleep(500);
                    ringFeederServo.setPosition(0.6);
                }
            }


            // shooter motor
            if (gamepad2.y) {
                if (ringShooterMotor1.getPower() == 0) {
                    ringShooterMotor1.setPower(-0.9);
                    ringShooterMotor2.setPower(-0.9);

                } else {
                    ringShooterMotor1.setPower(0);
                    ringShooterMotor2.setPower(0);
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

//        ringFeederServo.scaleRange(0.6,0.9); // sets min and max positions of servo
        ringFeederServo.setPosition(0.6); // sets initial position of servo

        // maps wobble goal servo variable to hardware configuration name
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");

        wobbleGoalServo.scaleRange(0.22,0.45); // sets min and max positions of servo
        wobbleGoalServo.setPosition(0); // sets initial position of servo

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // class to add and update telemetry
    private void addTelemetry() {
        telemetry.addData("gamepad1","");
        telemetry.addData("Left stick up/down"," Forward/backward");
        telemetry.addData("Left/right triggers"," Strafe left/right");
        telemetry.addData("Right stick left/right"," Rotate left/right");
        telemetry.addData("","");
        telemetry.addData("gamepad2","");
        telemetry.addData("A"," Ring Intake Motor On/Off");
        telemetry.addData("B"," Wobble Goal Servo");
        telemetry.addData("X"," Ring Feeder Servo");
        telemetry.addData("Y"," Ring Shooter Motors On/Off");
        telemetry.update();
    }
}