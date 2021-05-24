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

@Autonomous(name = "gyro tester 2.0", group = "") //name of the file

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
    private PIDController pidRotate;

    @Override
    public void runOpMode() {

        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
        SHOOTER = (DcMotorEx) (hardwareMap.dcMotor.get("SHOOTER"));
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
            // getting second reading and print the 2nd angle
            Orientation readingTwo = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angleZ = readingTwo.firstAngle;
            double angleY = readingTwo.secondAngle;
            double angleX = readingTwo.thirdAngle;
            telemetry.addData("Angle measure:", "" + angleZ + "," + angleY + "," + angleX);
            telemetry.update();


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

            // Set PID proportional value to start reducing power at about 50 degrees of rotation.
            // P by itself may stall before turn completed so we add a bit of I (integral) which
            // causes the PID controller to gently increase power if the turn is not completed.
            pidRotate = new PIDController(.003, .00003, 0);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
            telemetry.update();

            // wait for start button.

            waitForStart();

            telemetry.addData("Mode", "running");
            telemetry.update();

            sleep(1000);

            rotate(90, .5);

        }

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
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
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
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

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        telemetry.addData("Into the rotate function!", "");
        telemetry.update();
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            telemetry.addData("degrees are less than 0", "");
            telemetry.update();
            sleep(2000);
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                telemetry.addData("Getting off zero on right turn", "");
                telemetry.update();
                sleep(2000);
                RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
                LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
                RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
                LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

                //set motor powers:
                LEFTFRONT.setPower(.2);
                LEFTBACK.setPower(.2);
                RIGHTFRONT.setPower(.2);
                RIGHTBACK.setPower(.2);

                sleep(100);
            }
            telemetry.addData("angle = " + getAngle(),"");
            telemetry.update();
            sleep(2000);
            do
            {
                telemetry.addData("In the do part of right turn while loop, m_error = " + pidRotate.m_error, "");
                telemetry.update();
                sleep(2000);
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                telemetry.addData("power = " + power, "");
                telemetry.update();
                sleep(2000);
                RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
                LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
                RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
                LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

                //set motor powers:
                LEFTFRONT.setPower(power);
                LEFTBACK.setPower(power);
                RIGHTFRONT.setPower(power);
                RIGHTBACK.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                telemetry.addData("degree is greater than 0, left turn, m_error = " + pidRotate.m_error + "angle = " + getAngle(), "");
                telemetry.update();
                sleep(2000);
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
                LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
                RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
                LEFTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

                //set motor powers:
                LEFTFRONT.setPower(power);
                LEFTBACK.setPower(power);
                RIGHTFRONT.setPower(power);
                RIGHTBACK.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        LEFTFRONT.setPower(0);
        LEFTBACK.setPower(0);
        RIGHTFRONT.setPower(0);
        RIGHTBACK.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
        telemetry.addData("reset angle", "");
        telemetry.update();
        sleep(2000);
    }
}