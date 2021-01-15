package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends LinearOpMode {

    // sets variables for drive motors
/*    private DcMotor driveFL, driveFR, driveBL, driveBR;*/
    Servo wobbleGoalServo;

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

/*            // sets values of variables for gamepad1 (start+a) inputs
            forwardBackward = -gamepad1.left_stick_y;  // uses left stick to move forward and backward
            leftRight = gamepad1.right_trigger - gamepad1.left_trigger;    // uses triggers to strafe left and right
//          leftRight = gamepad1.left_stick_x;  // uses left stick to strafe left and right
            rotate = gamepad1.right_stick_x;  // uses right stick to rotate*/

           // driving in all directions and rotating
/*            driveFL.setPower(forwardBackward + leftRight + rotate);
            driveFR.setPower(forwardBackward - leftRight - rotate);
            driveBL.setPower(forwardBackward - leftRight + rotate);
            driveBR.setPower(forwardBackward + leftRight - rotate);*/

            // code for rotating wobble goal grabber
            if (gamepad2.b){
                if (wobbleGoalServo.getPosition() == 0){
                    wobbleGoalServo.setPosition(1);
                    wobbleGoalServo.setPosition(1);
                }
                else {
                    wobbleGoalServo.setPosition(0);
                    wobbleGoalServo.setPosition(0);
                }
            }


            // code for raising/lowering arm that grasps wobble goal

            // code for shooting rings

            // function to update telemetry
//            addTelemetry();

        }
    }

    // initialization class
    private void initialize() {

        // maps drive motor variables to hardware configuration names
/*        driveFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        driveFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        driveBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        driveBR = hardwareMap.get(DcMotor.class, "motorTestBR");*/

        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");
        wobbleGoalServo.scaleRange(0,0.22);

        // sets right motors to reverse direction so they're going the right way
/*        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);

        // sets drive motor zero power behavior to brake
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // class to add and update telemetry
    private void addTelemetry() {
/*        telemetry.addData("Front Left Drive Motor Power", driveFL.getPower());
        telemetry.addData("Front Right Drive Motor Power", driveFR.getPower());
        telemetry.addData("Back Left Drive Motor Power", driveBL.getPower());
        telemetry.addData("Back Right Drive Motor Power", driveBR.getPower());*/
        telemetry.update();

    }
}