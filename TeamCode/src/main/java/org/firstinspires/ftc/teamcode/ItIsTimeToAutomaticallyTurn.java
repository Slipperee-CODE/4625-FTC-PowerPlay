package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="ItIsTimeToAutomaticallyTurn", group="Linear Autonomous")
public class ItIsTimeToAutomaticallyTurn extends LinearOpMode
{
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private float SpeedReduction = 50;

    private BNO055IMU imu;



    private Orientation lastAngle = new Orientation();
    private double currentAngle = 0.0;



    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(50);


        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO0155IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu"); //We still need to name the imu on the robot this
        imu.initialize(parameters);


        SpeedReduction = SpeedReduction / 100;

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);



        while (!isStarted() && !isStopRequested()) //Init Loop
        {
            //Nothing rn ig
        }

        //Insert code to do stuff here

        Turn(90);

    }

    void PowerAllTheMotors(double RightFrontPower, double RightBackPower, double LeftFrontPower, double LeftBackPower)
    {
        RightFront.setPower(RightFrontPower);
        RightBack.setPower(RightBackPower);
        LeftFront.setPower(LeftFrontPower);
        LeftBack.setPower(LeftBackPower);
    }

    void ResetAngle()
    {
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    double GetAngle()
    {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;

        }
        else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currentAngle += deltaAngle;
        lastAngle = orientation;
        telemetry.addData("gyro rotation", orientation.firstAngle);
        return currentAngle;
    }

    void Turn (double degrees)
    {
        ResetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2){
            double motorPower = (error < 0 ? -0.2 : 0.2);
            PowerAllTheMotors(motorPower, motorPower, -motorPower, -motorPower);
            error = degrees - GetAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        PowerAllTheMotors(0,0,0,0);

    }

}


