/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.lang.*;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="! ! TheGoatedWoatedOpMoaded5.5", group="Iterative Opmode")
//@Disabled
public class TheGoatedWoatedOpMoaded extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double lastError = 0;

    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor spoolMotor = null;

    private Servo coneGrabber = null;
    //private CRServo coneGrabber = null;

    private float SpeedReduction = 50;

    private DistanceSensor distanceSensor;

    private double slideMotorPower = 0.6;

    private double armPosition;

    private final static double ARM_HOME = 0.0;
    private final static double ARM_MIN_RANGE = 0.0;
    private final static double ARM_MAX_RANGE = 1.0;
    private final double ARM_SPEED = 0.01;

    double y;
    double x;
    double rx;

    boolean DPAD_UP;
    boolean DPAD_DOWN;
    boolean DPAD_LEFT;
    boolean DPAD_RIGHT;

    boolean DPAD_UP2;
    boolean DPAD_DOWN2;
    boolean DPAD_LEFT2;
    boolean DPAD_RIGHT2;

    boolean gamepad1B;
    boolean gamepad2TopLeftTrigger;
    boolean gamepad2TopRightTrigger;

    boolean b2;

    int state = 0;

    double heightOfCone = 7;

    int cameraWidth = 640;

    int cameraHeight = 360;

    OpenCvWebcam webcam1 = null;



    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    int avgY;
    int avgX;

    int coneAlignmentConstant = 1;

    double intakeOffset = 0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        spoolMotor = hardwareMap.dcMotor.get("spoolMotor");



        coneGrabber = hardwareMap.servo.get("coneGrabber");
        //coneGrabber = hardwareMap.crservo.get("coneGrabber"); NEED TO FIND A Continuous SERVO?
        armPosition = 0.0f;

        SpeedReduction = SpeedReduction/100;



        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        //WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        webcam1.setPipeline(new TheGoatedWoatedOpMoaded.coneDetectingPipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        */

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        y = -gamepad1.left_stick_y; // Remember, this is reversed!
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        DPAD_UP = gamepad1.dpad_up;
        DPAD_DOWN = gamepad1.dpad_down;
        DPAD_LEFT = gamepad1.dpad_left;
        DPAD_RIGHT = gamepad1.dpad_right;

        DPAD_UP2 = gamepad2.dpad_up;
        DPAD_DOWN2 = gamepad2.dpad_down;
        DPAD_LEFT2 = gamepad2.dpad_left;
        DPAD_RIGHT2 = gamepad2.dpad_right;

        gamepad1B = gamepad1.b;
        gamepad2TopLeftTrigger = gamepad2.left_bumper;
        gamepad2TopRightTrigger = gamepad2.right_bumper;

        b2 = gamepad2.b;


        if (gamepad1B == true)
        {
            //StrafeToConeCode();
        }


        //distanceSensorConeCheckCode();




        grabberServoCode();
        linearSlideCode();
        movementCode();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }

    /*
    public void StrafeToConeCode()
    {
        double movementNum = autoOrientPIDControl((cameraWidth/2) - intakeOffset, avgX);


        if (movementNum > 0)
        {
            Move("left", Math.abs( (int) (movementNum) / coneAlignmentConstant), 0.6);
        }
        else
        {
            Move("right", Math.abs( (int) (movementNum) / coneAlignmentConstant), 0.6);
        }
    }
     */

    public double autoOrientPIDControl(double reference, double state)
    {
        double error = reference - state;
        integralSum += error * runtime.seconds();
        double derivative = (error - lastError) / runtime.seconds();
        lastError = error;

        runtime.reset();

        double output = (error + Kp) + (derivative * Kd) + (integralSum * Ki);


        return output;
    }

    public void distanceSensorConeCheckCode()
    {
        double distanceFromFloor = distanceSensor.getDistance(DistanceUnit.INCH);

        if (distanceFromFloor > heightOfCone)
        {
            telemetry.addData("Clear of Cone", distanceFromFloor);
            telemetry.update();
        }
        else
        {
            telemetry.addData("Would Hit Cone", distanceFromFloor);
            telemetry.update();
        }
    }

    public void grabberServoCode()
    {
        if (b2)
        {

            if (state == 1)
            {
                coneGrabber.setPosition(1);
                telemetry.addLine("Servo 1 Thing");
                telemetry.update();
            }
            else if (state == 0)
            {
                coneGrabber.setPosition(-1);
                telemetry.addLine("Servo 2 Thing");
                telemetry.update();
            }


            if (state < 1)
            {
                state++;
            }
            else
            {
                state = 0;
            }

            sleep(500);
        }
    }

    public void linearSlideCode()
    {
        if (DPAD_UP2){
            spoolMotor.setPower(-slideMotorPower);
        }
        else if (gamepad2TopLeftTrigger){
            spoolMotor.setPower(-slideMotorPower/2);
        }
        else if (gamepad2TopRightTrigger){
            spoolMotor.setPower(slideMotorPower/2);
        }
        else if (DPAD_DOWN2) {
            spoolMotor.setPower(slideMotorPower);
        }
        else if (!gamepad1.dpad_up && !gamepad2.dpad_down)
        {
            spoolMotor.setPower(0);
        }
    }

    public void movementCode()
    {
        if (DPAD_UP){
            y = Math.min(SpeedReduction,1);
        }
        else if (DPAD_DOWN){
            y = Math.max(-SpeedReduction,-1);
        }
        else if (DPAD_LEFT){
            x = Math.max(-SpeedReduction*1.2,-1);
        }
        else if (DPAD_RIGHT){
            x = Math.min(SpeedReduction*1.2,1);
        }
        
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        LeftFront.setPower(frontLeftPower*SpeedReduction);
        LeftBack.setPower(backLeftPower*SpeedReduction);
        RightFront.setPower(frontRightPower*SpeedReduction);
        RightBack.setPower(backRightPower*SpeedReduction);
    }

    public void Move(String direction, int milliseconds, double wheelPower){
        if (direction == "forward" || direction == "backward"){
            if (direction == "backward"){
                wheelPower = -wheelPower;
            }

            LeftFront.setPower(wheelPower);
            LeftBack.setPower(wheelPower);
            RightFront.setPower(wheelPower);
            RightBack.setPower(wheelPower);
            sleep(milliseconds);
            LeftFront.setPower(0);
            LeftBack.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);

        }
        else if (direction == "left" || direction == "right"){
            if (direction == "right"){
                wheelPower = -wheelPower;
            }

            LeftFront.setPower(wheelPower);
            LeftBack.setPower(-wheelPower);
            RightFront.setPower(-wheelPower);
            RightBack.setPower(wheelPower);
            sleep(milliseconds);
            LeftFront.setPower(0);
            LeftBack.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);
        }
    }

    public void sleep(int millis){
        try {
            Thread.sleep(millis);
        } catch (Exception e){}
    }

    public class coneDetectingPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // Make a working copy of the input matrix in HSV
            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value
            Scalar lowHSV1 = new Scalar(0, 100, 100);
            Scalar highHSV1 = new Scalar(10, 255, 255);
            Mat thresh = new Mat();

            Core.inRange(mat, lowHSV1, highHSV1, thresh);

            List<List<Integer>> coords = new ArrayList<>();

            for (int i = 0; i < cameraHeight; i++)
            {
                for (int j = 0; j < cameraWidth; j++)
                {
                    if (thresh.get(i, j)[0] >= 1)
                    {
                        int finalI = i;
                        int finalJ = j;

                        List<Integer> currentCoords = new ArrayList<Integer>() {{
                            add(finalI);
                            add(finalJ);
                        }};

                        coords.add(currentCoords);
                    }
                }
            }

            double totalX = 0;
            double totalY = 0;

            for (List<Integer> oneSetOfCoords : coords)
            {
                totalY += oneSetOfCoords.get(0);
                totalX += oneSetOfCoords.get(1);
            }


            avgY = (int) (totalY / coords.size());
            avgX = (int) (totalX / coords.size());



            return thresh;
        }
    }
}
