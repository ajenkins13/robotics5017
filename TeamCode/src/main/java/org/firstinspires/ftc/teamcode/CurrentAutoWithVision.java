//TODO(sachi): Rename this...

//comment out attachments since this is for this year
//FORWARD means REVERSE for the left wheels

package org.firstinspires.ftc.teamcode; //importing OUR package

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

import static java.lang.Math.abs;

//importing OpModes (linear and teleOp) and importing hardware (motors, sensors, servos)
//importing servos, motors, touch sensors

@Autonomous(name = "Current Auto with Vision", group = "") //name of the file

public class CurrentAutoWithVision extends LinearOpMode { //creating public class, extension of linear opmode

    //creating motors, touch sensors, and servos
    private DcMotor RIGHTFRONT;
    private DcMotor RIGHTBACK;
    private DcMotor LEFTFRONT;
    private DcMotor LEFTBACK;
    private DcMotorEx SHOOTER;
    private Servo FLICKER;
    private DcMotor WOBBLE;
    private Servo WOBBLEBLOCK;
    private DcMotor INTAKE;

    //Vision
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AZjoYn//////AAABmR/Gz+lF/k+IkcIQI/2fVYFiGF3UGkxilPyhwI/Cm4S3fmhAfVyLZconiBYacj0ZqCMizSfzW9evWjkqwZda2P4Z/lS48cBdkcOjXTviz930ACqjyN3S+Wep41I9xrmtZlv4t/X9cMUOdgQ22+AmBNZ3kTWFnl3PY2xBDsYSvqF1ifaU0R/LDlohIZhM4VBuMZWKLVX9cHOSwjti5T8lMmIMRX24RORLSSkL4ieJFRiusrOlAy6i+9s8io1KGfT4hKTk+LcUkCZUEbgANc8Srx76bhgnnerpMGmwuitHtDXTq8BFMey+fRMigGf+MwlL39A0qwFBq9wT45PdrX5y9+s1Huy0JkXZmt6uo1D8zRB1";
    //Gyro
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

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    final double encRotation = 537.6;

    private void zeroRings() {
        //Go forward to launch line
        ForwardForDistance(.5, 4.0);
        sleep(1000);
        //Shoot 3 rings
        shootRing();
        shootRing();
        shootRing();
        sleep(2000);
        SHOOTER.setPower(0);
        //Get in position for wobble
        ForwardForDistance(.5, .5);
        //Drop wobble goal in box 0
        dispenseWobble();
        //Reverse to position and turn to pick up wobble goal 2
        ForwardForDistance(0.5,-4);
        turnToAngle(90, .5);
        ForwardForDistance(.5, .5);
        turnToAngle(100, .5);
        sleep(1000);
        intakeWobble();
        //Squae back up and go forward to box to dispense wobble
        turnToAngle(-180, .5);
        ForwardForDistance(0.5, 4.5);
        dispenseWobble();
        WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
        WOBBLE.setPower(.7); //position = placeholder --> replace later after testing
    }

    private void oneRing() {
        turnAround();
        //Shoot 3 rings
        shootRing();
        shootRing();
        shootRing();
        sleep(2000);
        SHOOTER.setPower(0);
        //Turn on intake
        INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE.setPower(1);
        //Move backwards to pick up lone ring
        turnToAngle(-10, .5);
        ForwardForDistance(.5, -2);
        sleep(1000);
        //Return to launch line
        ForwardForDistance(.5, 2);
        sleep(1000);
        //Shoot 1 ring
        shootRing();
        sleep(2000);
        SHOOTER.setPower(0);
        //Turn and go forward to dispense wobble in box
        turnToAngle(35, .5);
        ForwardForDistance(.5, 1.5);
        dispenseWobble();
        ForwardForDistance(.5, -1.5);
    }

    private void fourRings() {
        turnAround();
        //Shoot 3 rings
        shootRing();
        shootRing();
        shootRing();
        sleep(2000);
        SHOOTER.setPower(0);
        //Go backwards to get near ring stack
        ForwardForDistance(.5, -1.7);
        sleep(1000);
        //Knock off the fourth ring
        INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE.setPower(1);
        sleep(2000);
        ForwardForDistance(.5, -.3);
        sleep(1000);
        //Return to launch line
        ForwardForDistance(.5, 2);
        sleep(1000);
        //Shoot 3 rings
        shootRing();
        shootRing();
        shootRing();
        sleep(2000);
        SHOOTER.setPower(0);
        //Go forward to dispense wobble
        ForwardForDistance(.5, 2);
        dispenseWobble();
        ForwardForDistance(.5, -2);
    }

    private void turnAround() {
        turnToAngle(40, .5);
        ForwardForDistance(.5, 2.5);//17 in
        turnToAngle(-80, .5);
        ForwardForDistance(.5, 2.5);
        turnToAngle(20, .5);
        //Go up to launch line
        ForwardForDistance(.5, .5);
    }

    private void dispenseWobble() {
        //drop the wobble goal in the launch line target zone
        WOBBLE.setDirection(DcMotorSimple.Direction.FORWARD);
        WOBBLE.setPower(1); //position = placeholder --> replace later after testing
        sleep(500);
        WOBBLE.setPower(0.5); //position = placeholder --> replace later after testing
        sleep(200);
        WOBBLE.setPower(0.4); //position = placeholder --> replace later after testing
        sleep(200);
        WOBBLE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WOBBLE.setPower(0);
        //open wobble block and sleep so that it opens before it moves
        WOBBLEBLOCK.setPosition(0);
        sleep(1000);
    }

    private void intakeWobble() {
        WOBBLEBLOCK.setPosition(1);
        sleep(1000);
        WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
        WOBBLE.setPower(.7); //position = placeholder --> replace later after testing
        sleep(500);
    }

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
        INTAKE = hardwareMap.dcMotor.get("INTAKE");

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        initVuforia();
        initTfod();
        //calibrate gyro and make sure its connected
        initGyro();

        waitForStart();

        if (opModeIsActive()) {

            //vuforia stuff - print out number of rings it sees
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.

            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 1.78 or 16/9).

                // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
                //tfod.setZoom(2.5, 1.78);
            }


            String numberRings = "None";
            int tries = 0;
            telemetry.addData("right before opModeIsActive", "");
            if (opModeIsActive()) {
                //goes forward before it looks for the rings
                ForwardForDistance(0.5,1.2);
                sleep(1000);

                while (true) {

                    if (tfod != null) {
                        telemetry.addData("tfod is not null", "");
                        telemetry.update();
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        tries++;
                        if (updatedRecognitions != null) {
                            telemetry.addData("updatedRecognitions has something", "");
                            telemetry.update();

                            if (updatedRecognitions.size() != 0) {
                                // step through the list of recognitions and display boundary info.
                                numberRings = (updatedRecognitions.get(0)).getLabel();
                                telemetry.addData(String.format("label (%d)", 0), numberRings);
                                telemetry.update();
                                break;
                            }
                        } else if (tries > 10000) {
                            telemetry.addData("breaking while loop cuz exceeded tries no ring", "");
                            telemetry.update();
                            break;
                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }

            SHOOTER.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SHOOTER.setVelocity(1845);   // Ticks per second.

            if (numberRings.equals("None")) {
                telemetry.addData("This is going to box A:", numberRings);
                telemetry.update();
                zeroRings();
            }
            else if (numberRings.equals("Single")) {
                telemetry.addData("This is going to box B:", numberRings);
                telemetry.update();
                oneRing();
            }
            else if (numberRings.equals("Quad")) {
                telemetry.addData("This is going to box C:", numberRings);
                telemetry.update();
                fourRings();
            }
            sleep(5000);

        }
    }

    private void shootRing() {
        FLICKER.setPosition(.2);
        sleep(500);
        FLICKER.setPosition(.7);
        sleep(2000);
    }

    private void stopEverything() {
        LEFTFRONT.setPower(0);
        RIGHTFRONT.setPower(0);
        LEFTBACK.setPower(0);
        RIGHTBACK.setPower(0);
    }

    private void ForwardForDistance(double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setTargetPosition(denc);
        LEFTBACK.setTargetPosition(denc);
        RIGHTBACK.setTargetPosition(denc);
        LEFTFRONT.setTargetPosition(denc);

        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);

        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }

        stopEverything();
    }

    private void TurnForDistance(double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setTargetPosition(denc);
        LEFTBACK.setTargetPosition(denc);
        RIGHTBACK.setTargetPosition(denc);
        LEFTFRONT.setTargetPosition(denc);

        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "running");
        telemetry.update();

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);
        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }
        stopEverything();
    }

    private void CrabForDistance(double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setTargetPosition(denc);
        LEFTBACK.setTargetPosition(denc);
        RIGHTBACK.setTargetPosition(denc);
        LEFTFRONT.setTargetPosition(denc);

        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "running");
        telemetry.update();

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);

        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }

        stopEverything();
    }

    private void resetEncoders() {
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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
        telemetry.addData("TURNING FUNCTION angle degree", globalHeading);
        telemetry.update();
        telemetry.addData("get angle", getAngle());
        telemetry.update();
        sleep(2000);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //If turning to a negative angle, turn right
        if (angle < 0) {
            while (getAngle() > angle) {
                //set motor directions:
                telemetry.addData("angle degree in right loop", globalHeading);
                telemetry.update();
                sleep(100);
                // Turn right.
                RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
                LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
                RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
                LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

                //set motor powers:
                LEFTFRONT.setPower(power);
                LEFTBACK.setPower(power);
                RIGHTFRONT.setPower(power);
                RIGHTBACK.setPower(power);
            }
        }
        //If turning to a positive angle, turn left
        else {
            while (getAngle() < angle) {
                //set motor directions:
                telemetry.addData("angle degree in left loop", globalHeading);
                telemetry.update();
                sleep(100);
                // Turn left.
                RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
                LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
                RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
                LEFTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

                //set motor powers:
                LEFTFRONT.setPower(power);
                LEFTBACK.setPower(power);
                RIGHTFRONT.setPower(power);
                RIGHTBACK.setPower(power);
            }
        }
        telemetry.addData("Broke loop", "");
        telemetry.update();
        LEFTFRONT.setPower(0);
        LEFTBACK.setPower(0);
        RIGHTFRONT.setPower(0);
        RIGHTBACK.setPower(0);
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