//TODO(sachi): Rename this...

//comment out attachments since this is for this year
//FORWARD means REVERSE for the left wheels

package org.firstinspires.ftc.teamcode; //importing OUR package

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//FROM CHAD

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//importing OpModes (linear and teleOp) and importing hardware (motors, sensors, servos)
//importing servos, motors, touch sensors

@Autonomous(name = "toz gyro tester", group = "") //name of the file

public class GyroTester extends LinearOpMode { //creating public class, extension of linear opmode

    Double width = 16.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement

    Double conversion = cpi * bias;
    Boolean exit = false;

    BNO055IMU imu;  // Reference to IMU device.
    Orientation angles;
    double globalHeading;
    Acceleration gravity;

    //creating motors, touch sensors, and servos
    private DcMotor RIGHTFRONT;
    private DcMotor RIGHTBACK;
    private DcMotor LEFTFRONT;
    private DcMotor LEFTBACK;
    private DcMotorEx SHOOTER;
    private Servo FLICKER;
    private DcMotor WOBBLE;
    private Servo WOBBLEBLOCK;

    @Override
    public void runOpMode() {

        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
        SHOOTER = (DcMotorEx)(hardwareMap.dcMotor.get("SHOOTER"));
        WOBBLE = hardwareMap.dcMotor.get("WOBBLE");
        WOBBLEBLOCK = hardwareMap.servo.get("WOBBLEBLOCK");
        FLICKER = hardwareMap.servo.get("FLICKER");

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            //calibrate gyro and make sure its connected
            initGyro();

            // getting first (initial) reading and print the angle
            Orientation initialReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double initialAngle = initialReading.firstAngle;
            telemetry.addData("Angle measure:", "" + initialAngle);
            telemetry.update();

            // time to manually reorient robot
            sleep(6000);

            // getting second reading and print the 2nd angle
            Orientation readingTwo = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angleZ = readingTwo.firstAngle;
            double angleY = readingTwo.secondAngle;
            double angleX = readingTwo.thirdAngle;
            telemetry.addData("Angle measure:", "" + angleZ + "," + angleY + "," + angleX);
            telemetry.update();
            sleep(12000);

            turnToAngle(90,0.5);
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {   // Are these values right?
        // Maybe need to determine AxesOrder from Control Hub's orientation.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalHeading = 0.0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = currentAngles.firstAngle - angles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalHeading += deltaAngle;

        angles = currentAngles;

        return globalHeading;
    }

    private void turnToAngle(double angle, double power) {
        resetAngle();
        while (getAngle() < angle) {
            //set motor directions:
            RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
            LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
            RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
            LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

            //set motor powers:
            LEFTFRONT.setPower(power);
            LEFTBACK.setPower(power);
            RIGHTFRONT.setPower(power);
            RIGHTBACK.setPower(power);
            sleep(100);
        }
    }

    /*
    This function is called at the beginning of the program to activate and calibrate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "Calibrating IMU...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

        LEFTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        LEFTFRONT.setPower(input);
        LEFTBACK.setPower(input);
        RIGHTFRONT.setPower(input);
        RIGHTBACK.setPower(input);
    }


}