package org.firstinspires.ftc.teamcode;

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="TheGoatedWoatedAutonomousModed", group="Iterative Autonomous")
//@Disabled
public class TheGoatedWoatedAutonomousModed extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;

    private float SpeedReduction = 50;

    OpenCvWebcam webcam1 = null;

    int colorIAmSeeing = 0;

    double actualAvg1;
    double actualAvg2;
    double actualAvg3;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        SpeedReduction = SpeedReduction/100;


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("G", actualAvg1);
        telemetry.addData("Y", actualAvg2);
        telemetry.addData("B", actualAvg3);


        if (actualAvg1 > 15 || actualAvg2 > 15 || actualAvg3 > 15){
            if (actualAvg1 > actualAvg2 && actualAvg1 > actualAvg3){
                colorIAmSeeing = 1;
            }
            else if (actualAvg2 > actualAvg1 && actualAvg2 > actualAvg3)
            {
                colorIAmSeeing = 2;
            }
            else if (actualAvg3 > actualAvg1 && actualAvg3 > actualAvg2){
                colorIAmSeeing = 3;
            }
            else{
                colorIAmSeeing = 0;
            }
        }

        if (colorIAmSeeing == 1){
            telemetry.addData("Green", colorIAmSeeing);
        }
        else if (colorIAmSeeing == 2){
            telemetry.addData("Yellow", colorIAmSeeing);
        }
        else if (colorIAmSeeing == 3){
            telemetry.addData("Blue", colorIAmSeeing);
        }
        else{
            telemetry.addData("No Color :( ", colorIAmSeeing);
        }

        telemetry.update();
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
        //Main loop

        if (colorIAmSeeing == 1){
            //Strafe left and forward



        }
        else if (colorIAmSeeing == 2){
            //Strafe right and forward



        }
        else if (colorIAmSeeing == 3){
            //Go forward



        }


        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        //Stuff for when the program stops

    }

    public class examplePipeline extends OpenCvPipeline{

        public Mat processFrame(Mat input) {
            // Make a working copy of the input matrix in HSV
            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);




            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value
            Scalar lowHSV1 = new Scalar(60, 100, 100);
            Scalar highHSV1 = new Scalar(71, 255, 255);
            Mat thresh = new Mat();



            Core.inRange(mat, lowHSV1, highHSV1, thresh);

            Scalar avg1 = Core.mean(thresh);

            actualAvg1 = avg1.val[0];




            Scalar lowHSV2 = new Scalar(20, 100, 100);
            Scalar highHSV2 = new Scalar(40, 255, 255);
            thresh = new Mat();



            Core.inRange(mat, lowHSV2, highHSV2, thresh);

            Scalar avg2 = Core.mean(thresh);

            actualAvg2 = avg2.val[0];



            Scalar lowHSV3 = new Scalar(96, 100, 100);
            Scalar highHSV3 = new Scalar(121, 255, 255);
            thresh = new Mat();


            Core.inRange(mat, lowHSV3, highHSV3, thresh);

            Scalar avg3 = Core.mean(thresh);

            actualAvg3 = avg3.val[0];


            return thresh;
        }
    }
}


