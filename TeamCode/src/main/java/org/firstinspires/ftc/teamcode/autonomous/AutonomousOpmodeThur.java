package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Autonomous Thursday", group = "lmsbots")

public class AutonomousOpmodeThur extends LinearOpMode {

    //  sets variables for drive motors, IMU, etc.
    DcMotor driveFL, driveFR, driveBL, driveBR, takingInRingsMotor, ringShooterMotor1, ringShooterMotor2;
    Servo ringFeederServo, wobbleGoalServo;
    BNO055IMU imu;
    OpenCvWebcam webcam;
    WebcamName webcamName;
    RingDeterminationPipeline pipeline;
    double drivePower = 1.0, turnPower = 0.6;
    double wheelDiameter = 3.77953;
    double encoderTicksPerRotation = 537.6;
    double gearRatio = 2.0;
    double wheelCircumference = wheelDiameter * Math.PI;
    double encoderTicksPerInch = ((encoderTicksPerRotation / wheelCircumference) * gearRatio);
    double robotHeading = 0;
    int xPos = 48, yPos = 0;
    int ringNumber;

    // called when the initialization button is  pressed
    @Override
    public void runOpMode() {

        // method to initialize robot
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        if (opModeIsActive()) {

            ringNumber = pipeline.ringNumber;

            driveToBasic(36,59);
        }
    }

    private void shootRingStack() {
        driveToBasic(32,59);

        for (int i = 0; i < 5; i++) {
            ringFeederServo.setPosition(1.0);
            sleep(400);
            ringFeederServo.setPosition(0.6);
            sleep(400);
        }
    }

    private void pickupRingStack() {
        takingInRingsMotor.setPower(-1.0);
//        driveToAdvanced(48,50);
        driveToBasic(51,50);
        driveToBasic(51,40);
    }

    private void parkOverLaunchLine() {
//         driveToAdvanced(36,72);
        driveToBasic(36,70);
    }

    private void shootRings() {
        // start ring shooter motor
        ringShooterMotor1.setPower(-0.9);
        ringShooterMotor2.setPower(-0.9);

        // drive to shooting area
//        driveToAdvanced(34,61);
        driveToBasic(36,59);

        for (int i = 0; i < 5; i++) {
            ringFeederServo.setPosition(1.0);
            sleep(400);
            ringFeederServo.setPosition(0.6);
            sleep(400);
        }
    }

    private void getSecondWobbleGoal() {
        driveToAdvanced(36,5);
        driveToBasic(60,5);
        drivePower = 0.2;
        driveToBasic(60,20);
        drivePower = 1.0;

        if (ringNumber == 0) {
            // drop off wobble goal at target zone A
            driveToAdvanced(55,70);
//        driveToBasic(54,70);

            sleep(1000);
            driveToBasic(54,55);
        }
        else if (ringNumber == 1) {
            // drop off wobble goal at target zone B
            driveToAdvanced(36,88);
            sleep(1000);
            driveToBasic(36, 73);
        }
        else {
            // drop off wobble goal at target zone C
            driveToAdvanced(54,94);
            sleep(1000);
            driveToBasic(54,79);
        }

    }

    private void dropOffFirstWobbleGoal() {

        wobbleGoalServo.setPosition(0.45);

        if (ringNumber == 0) {
            // drop off wobble goal at target zone A
            driveToAdvanced(60,76);
            sleep(1000);
            driveToBasic(60,60);
        }
        else if (ringNumber == 1) {
            // drop off wobble goal at target zone B
            driveToAdvanced(36,88);
            sleep(1000);
            driveToBasic(36, 73);
        }
        else {
            // drop off wobble goal at target zone C
            driveToAdvanced(54,94);
            sleep(1000);
            driveToBasic(54,79);
        }
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        driveFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        driveFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        driveBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        driveBR = hardwareMap.get(DcMotor.class, "motorTestBR");

        // sets two motors to go in reverse so that robot runs right
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        // sets all motors to brake when you put zero power
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ringShooterMotor1 = hardwareMap.get(DcMotor.class, "ringShooterMotor1");
        ringShooterMotor2 = hardwareMap.get(DcMotor.class, "ringShooterMotor2");
        takingInRingsMotor = hardwareMap.get(DcMotor.class, "takingInRingsMotor");

        ringFeederServo = hardwareMap.get(Servo.class, "ringFeederServo");
        ringFeederServo.setPosition(0.6);

        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");
        //wobbleGoalServo.scaleRange(0.22,0.45);
        wobbleGoalServo.setPosition(0.1);

        // initialize webcam
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        // initialize imu
/*        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);*/

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // this method determines how many rings there are at the start of the game - 0, 1 or 4
    private int determineRingNumber() {
        int ringNumber1, ringNumber2, ringNumber3, ringNumber4, finalRingNumber;

        sleep(3000);
        if (pipeline.ringNumber == 4) {
            ringNumber1 = 4;
        }
        else if (pipeline.ringNumber == 1) {
            ringNumber1 = 1;
        }
        else {
            ringNumber1 = 0;
        }
        sleep(250);
        if (pipeline.ringNumber == 4) {
            ringNumber2 = 4;
        }
        else if (pipeline.ringNumber == 1) {
            ringNumber2 = 1;
        }
        else {
            ringNumber2 = 0;
        }
        sleep(250);
        if (pipeline.ringNumber == 4) {
            ringNumber3 = 4;
        }
        else if (pipeline.ringNumber == 1) {
            ringNumber3 = 1;
        }
        else {
            ringNumber3 = 0;
        }
        sleep(250);
        if (pipeline.ringNumber == 4) {
            ringNumber4 = 4;
        }
        else if (pipeline.ringNumber == 1) {
            ringNumber4 = 1;
        }
        else {
            ringNumber4 = 0;
        }

        finalRingNumber = (int)(((ringNumber1 + ringNumber2 + ringNumber3 + ringNumber4)/4));
        if (finalRingNumber == 2){
            finalRingNumber = 1;
        }
        else if (finalRingNumber == 3){
            finalRingNumber = 4;
        }
        return finalRingNumber;
    }

    // this method drives autonomously by strafing left/right and then moving forward/backward
    private void driveToBasic(int xTarget, int yTarget) {

        // determine distance needed to travel in both directions
        int xDiff = xTarget - xPos;
        int yDiff = yTarget - yPos;

        // move the robot
        strafeRight(xDiff);
        driveForward(yDiff);

        // update the current position of the robot
        xPos = xTarget;
        yPos = yTarget;
    }

    private void driveForward(double inches) {

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
        driveFR.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveBL.setTargetPosition((int)(inches * encoderTicksPerInch));
        driveBR.setTargetPosition((int)(inches * encoderTicksPerInch));

        driveFL.setPower(drivePower);
        driveFR.setPower(drivePower);
        driveBL.setPower(drivePower);
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

    // this method drives autonomously by pointing toward/away from a target and driving forward/backward
    private void driveToIntermediate(int xTarget, int yTarget) {

        int xDiff = xTarget - xPos;
        int yDiff = yTarget - yPos;

        // determine distance needed to travel using pythagorean theorem
        double distance = Math.sqrt((xDiff*xDiff)+(yDiff*yDiff));

        double targetHeading;
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = orientation.firstAngle;

        if (xDiff == 0){
            driveForward(yDiff);
        }
        else if (yDiff == 0){
            strafeRight(xDiff);
        }

        else if (xDiff > 0){
            if (yDiff > 0){
                targetHeading = Math.toDegrees(Math.atan2(xDiff,yDiff));

                telemetry.addData("Going to"," " + xTarget + ", " + yTarget);
                telemetry.addData("targetHeading",targetHeading);
                telemetry.addData("distance",(int)distance);
                telemetry.update();
                sleep(2000);

                turnRight(targetHeading);
                driveForward(distance);
                turnLeft(targetHeading);
            }
            else {
                targetHeading = 180 - Math.toDegrees(Math.atan2(xDiff,yDiff));

                telemetry.addData("Going to"," " + xTarget + ", " + yTarget);
                telemetry.addData("targetHeading",targetHeading);
                telemetry.addData("distance",(int)distance);
                telemetry.update();
                sleep(3000);

                turnLeft(targetHeading);
                driveForward(-distance);
                turnRight(targetHeading);
            }

        }
        else if (yDiff > 0){
            targetHeading = -Math.toDegrees(Math.atan2(xDiff,yDiff));

            telemetry.addData("Going to"," " + xTarget + ", " + yTarget);
            telemetry.addData("targetHeading",targetHeading);
            telemetry.addData("distance",(int)distance);
            telemetry.update();
            sleep(3000);

            turnLeft(targetHeading);
            driveForward(distance);
            turnRight(targetHeading);
        }
        else {
            targetHeading = 180 + Math.toDegrees(Math.atan2(xDiff,yDiff));

            telemetry.addData("Going to"," " + xTarget + ", " + yTarget);
            telemetry.addData("targetHeading",targetHeading);
            telemetry.addData("distance",(int)distance);
            telemetry.update();
            sleep(3000);

            turnRight(targetHeading);
            driveForward(-distance);
            turnLeft(targetHeading);
        }

        xPos = xTarget;
        yPos = yTarget;

    }

    // this method drives autonomously by strafing/driving wherever it needs to go while facing forward the whole time
    private void driveToAdvanced(int xTarget, int yTarget) {
        int xDiff = xTarget - xPos;
        int yDiff = yTarget - yPos;
        double heading;

        if (xDiff >=0){
            heading = Math.toDegrees(Math.atan2(xDiff,yDiff));
        }
        else if (yDiff < 0){
            heading = 180 + Math.toDegrees(Math.atan2(Math.abs(xDiff), Math.abs(yDiff)));
        }
        else {
            heading = 270 + Math.toDegrees(Math.atan2(Math.abs(yDiff), Math.abs(xDiff)));
        }

        // determine distance needed to travel using pythagorean theorem
        double distance = Math.sqrt((xDiff*xDiff)+(yDiff*yDiff));

        double encoderTicks = encoderTicksPerInch * (Math.abs(xDiff) + Math.abs(yDiff));
        telemetry.addData("distance",(int)distance);
        telemetry.addData("heading",heading);
        telemetry.addData("encoderTicksPerInch",encoderTicksPerInch);
        telemetry.update();
        sleep(3000);

        encoderDrive(encoderTicks, drivePower, heading);
        xPos = xTarget;
        yPos = yTarget;
    }

    // method for driving by encoder
    private void encoderDrive(double encoderTicks, double power, double heading) {

        double powerFLBR = (power*FLBRpowerRatio(heading));
        double powerFRBL = (power*FRBLpowerRatio(heading));

        double encoderFLBR = encoderTicks*powerFLBR;
        double encoderFRBL = encoderTicks*powerFRBL;

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

        driveFL.setTargetPosition((int)encoderFLBR);
        driveFR.setTargetPosition((int)encoderFRBL);
        driveBL.setTargetPosition((int)encoderFRBL);
        driveBR.setTargetPosition((int)encoderFLBR);

        driveFL.setPower(powerFLBR);
        driveFR.setPower(powerFRBL);
        driveBL.setPower(powerFRBL);
        driveBR.setPower(powerFLBR);

        while (opModeIsActive() && (driveFL.isBusy() || driveFR.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", driveFL.getCurrentPosition() + "  busy=" + driveFL.isBusy());
            telemetry.addData("encoder-back-left", driveBL.getCurrentPosition() + "  busy=" + driveFL.isBusy());
            telemetry.addData("encoder-fwd-right", driveFR.getCurrentPosition() + "  busy=" + driveFR.isBusy());
            telemetry.addData("encoder-back-right", driveBR.getCurrentPosition() + "  busy=" + driveFR.isBusy());
            telemetry.update();
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

        resetStartTime();
    }

    double FLBRpowerRatio(double heading){
        double powerRatio=0;

        while(heading>=360){
            heading = heading - 360;
        }

        if (heading>=0 && heading <= 90){
            powerRatio = 1;
        }
        else if (heading >= 180 && heading <=270){
            powerRatio = -1;
        }
        else if (heading == 135 || heading == 315){
            powerRatio = 0;
        }
        else if ((heading > 90 && heading < 135)){
            heading = heading - 90;
            powerRatio = 1-(heading/45);
        }
        else if ((heading > 315 && heading < 360)){
            heading = heading - 315;
            powerRatio = heading/45;
        }
        else if ((heading > 135 && heading < 180)){
            heading = heading - 135;
            powerRatio = -1 * (heading/45);
        }
        else if ((heading > 270 && heading < 315)){
            heading = heading - 270;
            powerRatio = (-1 * (1-(heading/45)));
        }

        return powerRatio;
    }

    double FRBLpowerRatio(double heading){
        double powerRatio=0;

        while(heading>=360){
            heading = heading - 360;
        }

        if (heading==0 || (heading >= 270 && heading < 360)){
            powerRatio=1;
        }
        else if (heading >= 90 && heading <=180){
            powerRatio = -1;
        }
        else if (heading == 45 || heading == 225) {
            powerRatio = 0;
        }
        else if ((heading > 0 && heading < 45)){
            powerRatio = 1-(heading/45);
        }
        else if ((heading > 225 && heading < 270)){
            heading = heading - 225;
            powerRatio = heading/45;
        }
        else if ((heading > 45 && heading < 90)){
            heading = heading - 45;
            powerRatio = -1 * (heading/45);
        }
        else if ((heading > 180 && heading < 225)){
            heading = heading - 180;
            powerRatio = (-1 * (1-(heading/45)));
        }

        return powerRatio;
    }

    private void turnLeft(double degrees) {
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (robotHeading + degrees > 180) {
            while (opModeIsActive() && Angle() > -(360 - (robotHeading + degrees))){
                driveFL.setPower(-turnPower);
                driveFR.setPower(turnPower);
                driveBL.setPower(-turnPower);
                driveBR.setPower(turnPower);
            }
            while (opModeIsActive() && Angle() < -(360 - (robotHeading + degrees))){
                driveFL.setPower(-turnPower);
                driveFR.setPower(turnPower);
                driveBL.setPower(-turnPower);
                driveBR.setPower(turnPower);
            }
        }
        else {
            while (opModeIsActive() && Angle() < (robotHeading + degrees)){
                driveFL.setPower(-turnPower);
                driveFR.setPower(turnPower);
                driveBL.setPower(-turnPower);
                driveBR.setPower(turnPower);
            }
        }

        robotHeading = Angle();

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);
    }

    private void turnRight(double degrees) {
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (robotHeading - degrees > -180) {
            while (opModeIsActive() && Angle() > (robotHeading - degrees)){
                driveFL.setPower(turnPower);
                driveFR.setPower(-turnPower);
                driveBL.setPower(turnPower);
                driveBR.setPower(-turnPower);
            }
        }
        else {
            while (opModeIsActive() && Angle() < (360 + (robotHeading - degrees))){
                driveFL.setPower(turnPower);
                driveFR.setPower(-turnPower);
                driveBL.setPower(turnPower);
                driveBR.setPower(-turnPower);
            }
            while (opModeIsActive() && Angle() > (360 + (robotHeading - degrees))){
                driveFL.setPower(turnPower);
                driveFR.setPower(-turnPower);
                driveBL.setPower(turnPower);
                driveBR.setPower(-turnPower);
            }
        }

        robotHeading = Angle();

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

    }

    private double Angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        public int ringNumber;

        // Color constant to draw red box around captured image
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        // Values that define the region we want to capture and analyze
        static final Point TOP_LEFT_ANCHOR_POINT = new Point(175,180);

        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                TOP_LEFT_ANCHOR_POINT.x,
                TOP_LEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Variables for matrices that will hold information about our defined region of the image captured
        Mat region1_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        int avg1;

        // This method converts the image from RGB to YCrCb and extracts the Cr portion of it to the Cr variable
        void inputToCr(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCr(firstFrame);

            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCr(input);

            avg1 = (int) Core.mean(region1_Cr).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    0); // Zero thickness is clear shape with outline; -1 is filled in

            if(avg1 > FOUR_RING_THRESHOLD){
                ringNumber = 4;
            }else if (avg1 > ONE_RING_THRESHOLD){
                ringNumber = 1;
            }else{
                ringNumber = 0;
            }

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

}