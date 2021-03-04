package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Teleop extends LinearOpMode {

    // sets variables for motors and servos
    private DcMotor driveFL, driveFR, driveBL, driveBR, takingInRingsMotor, ringShooterMotor1, ringShooterMotor2;
    Servo wobbleGoalServo, ringFeederServo, wobbleGoalReleaseServo;
    BNO055IMU imu;
    Orientation angles;
    double towerGoalRingMotorSpeed = -0.85, powerShotRingMotorSpeed = -0.7;
    double wobbleGoalStartingPosition = 0.1, wobbleGoalVerticalPosition = 0.22, wobbleGoalReleasePosition = 0.45;

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
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // function to update telemetry
            addTelemetry();

           // driving in all directions and rotating
            driveFL.setPower(forwardBackward + leftRight + rotate);
            driveFR.setPower(forwardBackward - leftRight - rotate);
            driveBL.setPower(forwardBackward - leftRight + rotate);
            driveBR.setPower(forwardBackward + leftRight - rotate);

            // Press A for ring intake motor
            if (gamepad2.a) {
                if (takingInRingsMotor.getPower() < 0){
                    takingInRingsMotor.setPower(1);
                }
                else {
                    takingInRingsMotor.setPower(-1);
                }
            }

            // Right stick for wobble goal servo
            while (gamepad2.right_stick_y < 0){
                wobbleGoalServo.setPosition(wobbleGoalReleasePosition);
            }
            while (gamepad1.right_stick_y > 0) {
                    wobbleGoalServo.setPosition(wobbleGoalVerticalPosition);
            }

            // Press right trigger for ring feeder servo
            if (gamepad2.right_trigger == 1) {
                if (ringShooterMotor1.getPower() <= powerShotRingMotorSpeed){
                    ringFeederServo.setPosition(0.9);
                    sleep(300);
                    ringFeederServo.setPosition(0.6);
                }
            }

            // Press left trigger to unlock and lock wobble goal release servo
/*            if (gamepad2.left_trigger == 1) {
                if (wobbleGoalReleaseServo.getPosition() == 0) {
                    wobbleGoalReleaseServo.setPosition(0.5);
                }
                else {
                    wobbleGoalReleaseServo.setPosition(0);
                }
            }*/

            // press Y for shooter motor for tower goal
            if (gamepad2.y) {
                if (ringShooterMotor1.getPower() < powerShotRingMotorSpeed) {
                    ringShooterMotor1.setPower(0);
                    ringShooterMotor2.setPower(0);

                } else {
                    ringShooterMotor1.setPower(towerGoalRingMotorSpeed);
                    ringShooterMotor2.setPower(towerGoalRingMotorSpeed);
                }
            }

            // press X for shooter motor for powershot
            if (gamepad2.x) {
                ringShooterMotor1.setPower(powerShotRingMotorSpeed);
                ringShooterMotor2.setPower(powerShotRingMotorSpeed);
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

//        ringFeederServo.scaleRange(0.6,0.9); // sets min and max positions of servo
        ringFeederServo.setPosition(0.6); // sets initial position of servo

        // maps wobble goal servo variable to hardware configuration name
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");
        wobbleGoalServo.setPosition(wobbleGoalStartingPosition); // sets initial position of servo
//        wobbleGoalReleaseServo = hardwareMap.get(Servo.class, "wobbleGoalReleaseServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // class to add and update telemetry
    private void addTelemetry() {
        telemetry.addData("Robot heading", angles.firstAngle);
        telemetry.update();
    }
}