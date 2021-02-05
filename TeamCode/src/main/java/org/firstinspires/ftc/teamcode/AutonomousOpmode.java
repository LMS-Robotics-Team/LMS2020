package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name = "Autonomous", group = "lmsbots")

public class AutonomousOpmode extends LinearOpMode {

    // sets variables for drive motors, IMU, etc.
    DcMotor driveFL, driveFR, driveBL, driveBR;
    BNO055IMU imu;
    OpenCvWebcam webcam;
    WebcamName webcamName;
    RingDeterminationPipeline pipeline;
    double drivePower = 0.8, turnPower = 0.6;
    double wheelDiameter = 3.77953;
    double encoderTicksPerRotation = 537.6;
    double wheelCircumference = wheelDiameter * Math.PI;
    double encoderTicksPerInch = encoderTicksPerRotation / wheelCircumference;
    double robotHeading = 0;
    int xPos = 0, yPos = 0;
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

            // autonomous code goes here

            dropOffFirstWobbleGoal();
            dropOffSecondWobbleGoal();
            shootPreLoadedRings();
            shootRingStack();
            parkOverLaunchLine();

//            driveToBasic(24, 24);  // drives forward/backward and strafes left/right
//            driveToIntermediate(24, 24);  // rotates toward/away from target and drives forward/backward
//            driveToAdvanced(24, 24); // strafes in any direction to target

            while (opModeIsActive())
            {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Ring Number", ringNumber);
                telemetry.addData("X position", xPos);
                telemetry.addData("Y position", yPos);
                telemetry.update();

                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
            }

        }

    }

    private void shootRingStack() {
        // if there is 1 or 4 rings in stack, go to stack (otherwise don't do anything)
        // turn on ring intake motor
        // drive to shooting area
        // shoot rings
    }

    private void parkOverLaunchLine() {
        // drive to launch line
    }

    private void shootPreLoadedRings() {
        // drive to shooting area
        // shoot rings
    }

    private void dropOffSecondWobbleGoal() {
        // drive to second wobble goal
        // pickup wobble goal
        // drive to correct target zone
        // drop off wobble goal

    }

    private void dropOffFirstWobbleGoal() {
        sleep(500);
        ringNumber = pipeline.ringNumber;

        // drive to correct target zone
        // drop off wobble goal

    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // map variables to robot components
/*        driveFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        driveFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        driveBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        driveBR = hardwareMap.get(DcMotor.class, "motorTestBR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");*/
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

/*        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    // this method drives autonomously by going forward/backward and strafing left/right
    private void driveToBasic(int xTarget, int yTarget) {
        int xDiff = xTarget - xPos;
        int yDiff = yTarget - yPos;

        // move the robot
        strafeRight(xDiff);
        driveForward(yDiff);

        // update the current position of the robot
        xPos = xTarget;
        yPos = yTarget;
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

        encoderDrive(encoderTicks, drivePower , heading);
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

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        public int ringNumber;

        // Color constant to draw red box around captured image
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        // Values that define the region we want to capture and analyze
        static final Point TOP_LEFT_ANCHOR_POINT = new Point(120,95);

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