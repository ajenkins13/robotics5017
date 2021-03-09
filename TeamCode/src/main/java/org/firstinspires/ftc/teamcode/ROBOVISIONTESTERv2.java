///* Copyright (c) 2019 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
///**
// * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
// * determine the position of the Ultimate Goal game elements.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
// *
// * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
// * is explained below.
// */
//@Autonomous(name = "current vision program", group = "Concept")
////@Disabled
//public class ROBOVISIONTESTERv2 extends LinearOpMode {
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
//
//    private DcMotor RIGHTFRONT;
//    private DcMotor RIGHTBACK;
//    private DcMotor LEFTFRONT;
//    private DcMotor LEFTBACK;
//    private DcMotor SHOOTER;
//    private Servo FLICKER;
//    private DcMotor WOBBLE;
//    private Servo WOBBLEBLOCK;
//    private DcMotor INTAKE;
//
//    final double encRotation = 537.6;
//
//    /*
//     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//     * web site at https://developer.vuforia.com/license-manager.
//     *
//     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//     * random data. As an example, here is a example of a fragment of a valid key:
//     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//     * Once you've obtained a license key, copy the string from the Vuforia web site
//     * and paste it in to your code on the next line, between the double quotes.
//     */
//    private static final String VUFORIA_KEY =
//            "AZjoYn//////AAABmR/Gz+lF/k+IkcIQI/2fVYFiGF3UGkxilPyhwI/Cm4S3fmhAfVyLZconiBYacj0ZqCMizSfzW9evWjkqwZda2P4Z/lS48cBdkcOjXTviz930ACqjyN3S+Wep41I9xrmtZlv4t/X9cMUOdgQ22+AmBNZ3kTWFnl3PY2xBDsYSvqF1ifaU0R/LDlohIZhM4VBuMZWKLVX9cHOSwjti5T8lMmIMRX24RORLSSkL4ieJFRiusrOlAy6i+9s8io1KGfT4hKTk+LcUkCZUEbgANc8Srx76bhgnnerpMGmwuitHtDXTq8BFMey+fRMigGf+MwlL39A0qwFBq9wT45PdrX5y9+s1Huy0JkXZmt6uo1D8zRB1";
//
//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
//    private VuforiaLocalizer vuforia;
//
//    /**
//     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//     * Detection engine.
//     */
//    private TFObjectDetector tfod;
//
//    @Override
//    public void runOpMode() {
//        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//        // first.
//
//        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
//        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
//        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
//        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
//        SHOOTER = hardwareMap.dcMotor.get("SHOOTER");
//        WOBBLE = hardwareMap.dcMotor.get("WOBBLE");
//        WOBBLEBLOCK = hardwareMap.servo.get("WOBBLEBLOCK");
//        INTAKE = hardwareMap.dcMotor.get("INTAKE");
//        FLICKER = hardwareMap.servo.get("FLICKER");
//
//        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
//        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        initVuforia();
//        initTfod();
//
//        /**
//         * Activate TensorFlow Object Detection before we wait for the start command.
//         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
//         **/
//        if (tfod != null) {
//            tfod.activate();
//
//            // The TensorFlow software will scale the input images from the camera to a lower resolution.
//            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
//            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
//            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
//            // should be set to the value of the images used to create the TensorFlow Object Detection model
//            // (typically 1.78 or 16/9).
//
//            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
//            //tfod.setZoom(2.5, 1.78);
//        }
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//        waitForStart();
//
//        if (opModeIsActive()) {
//
////            //crab left
////            CrabForDistance(1, 0.5);
////
////            //move backwards to reach the rings
////            ForwardForDistance(0.5,);
//
//            //reach the rings
//            CrabForDistance(0.2, 0.5);
//            sleep(500);
//            ForwardForDistance(0.5, -1);
//            sleep(1000);
//
//            //read the rings
//            String readLabel = "";
//
//            for (int count = 0; count < 1000; count++) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                            readLabel = recognition.getLabel();
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                    recognition.getRight(), recognition.getBottom());
//                        }
//                        telemetry.update();
//                    }
//                }
//                sleep(10);
//            }
//            sleep(1000);
//
//
//            //move to the launch line
//
//            CrabForDistance(0.2, 0.5);
//            sleep(500);
//            ForwardForDistance(0.5, -3);
//            sleep(1000);
//
//            //shoot 3 rings at the high goal
//            SHOOTER.setDirection(DcMotorSimple.Direction.REVERSE);
//            SHOOTER.setPower(1); //maybe max?
//            sleep(2000);
//            FLICKER.setPosition(.2);
//            sleep(500);
//            FLICKER.setPosition(1);
//
//            sleep(1000);
//            FLICKER.setPosition(.2);
//            sleep(500);
//            FLICKER.setPosition(1);
//
//            sleep(1000);
//            FLICKER.setPosition(.2);
//            sleep(500);
//            FLICKER.setPosition(1);
//            sleep(500);
//            FLICKER.setPosition(.2);
//
//            //use the data gathered
//
//            // 4 rings spotted
//            if (readLabel.equals("Quad")) {
//                //move to square C (furthest)
//            }
//            // 1 ring spotted
//            else if (readLabel.equals("Single")) {
//                //move to square B (middle)
//                //move to square A (closest)
//                //move over to the target square
//                //move forward
//                ForwardForDistance(0.5, -1.5);
//                sleep(500);
//                //crab left to the target zone
//                CrabForDistance(1, 1);
//
//                //drop the wobble goal in the launch line target zone
//                WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
//                WOBBLE.setPower(0.8); //position = placeholder --> replace later after testing
//                sleep(200);
//                WOBBLE.setPower(0.5); //position = placeholder --> replace later after testing
//                sleep(200);
//                WOBBLE.setPower(0.4); //position = placeholder --> replace later after testing
//                sleep(200);
//                WOBBLE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                WOBBLE.setPower(0);
//
//                sleep(2000);
//                WOBBLEBLOCK.setPosition(0);
//                sleep(1000);
//
//                ForwardForDistance(0.3,1);
//
//                //park on the launch line (not touching the wobble goal)
//                //crab right
//                CrabForDistance(0.5,-1);
//
//                ForwardForDistance(0.5, -1);
//            }
//            // 0 rings spotted
//            else {
////                //move to square A (closest)
////                //move over to the target square
////                //move forward
////                ForwardForDistance(0.5, -1.5);
////                sleep(500);
////                //crab left to the target zone
////                CrabForDistance(1, 1);
////
////                //drop the wobble goal in the launch line target zone
////                WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
////                WOBBLE.setPower(0.8); //position = placeholder --> replace later after testing
////                sleep(200);
////                WOBBLE.setPower(0.5); //position = placeholder --> replace later after testing
////                sleep(200);
////                WOBBLE.setPower(0.4); //position = placeholder --> replace later after testing
////                sleep(200);
////                WOBBLE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////                WOBBLE.setPower(0);
////
////                sleep(2000);
////                WOBBLEBLOCK.setPosition(0);
////                sleep(1000);
////
////                ForwardForDistance(0.3,1);
////
////                //park on the launch line (not touching the wobble goal)
////                //crab right
////                CrabForDistance(0.5,-1);
////
////                ForwardForDistance(0.5, -1);
//            }
//
//
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }
//    }
//
//    /**
//     * Initialize the Vuforia localization engine.
//     */
//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//    }
//
//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.8f;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//    }
//
//    private void stopEverything() {
//        LEFTFRONT.setPower(0);
//        RIGHTFRONT.setPower(0);
//        LEFTBACK.setPower(0);
//        RIGHTBACK.setPower(0);
//    }
//
//    private void ForwardForDistance(double power, double revolutions) {
//        int denc = (int)Math.round(revolutions * encRotation);
//
//        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
//        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
//        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
//        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        RIGHTFRONT.setTargetPosition(denc);
//        LEFTBACK.setTargetPosition(denc);
//        RIGHTBACK.setTargetPosition(denc);
//        LEFTFRONT.setTargetPosition(denc);
//
//        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        telemetry.addData("Mode", "running");
//        telemetry.update();
//
//        RIGHTFRONT.setPower(power);
//        LEFTFRONT.setPower(power);
//        RIGHTBACK.setPower(power);
//        LEFTBACK.setPower(power);
//
//        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//        {
//            telemetry.addData("encoder-back-left", LEFTBACK.getCurrentPosition() + "  busy=" + LEFTBACK.isBusy());
//            telemetry.addData("encoder-forward-left", LEFTFRONT.getCurrentPosition() + "  busy=" + LEFTFRONT.isBusy());
//            telemetry.addData("encoder-back-right", RIGHTBACK.getCurrentPosition() + "  busy=" + RIGHTBACK.isBusy());
//            telemetry.addData("encoder-forward-right", RIGHTFRONT.getCurrentPosition() + "  busy=" + RIGHTFRONT.isBusy());
//
//            telemetry.update();
//            idle();
//        }
//
//        stopEverything();
//    }
//
//    private void TurnForDistance(double power, double revolutions) {
//        int denc = (int)Math.round(revolutions * encRotation);
//
//        RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
//        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
//        RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
//        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        RIGHTFRONT.setTargetPosition(denc);
//        LEFTBACK.setTargetPosition(denc);
//        RIGHTBACK.setTargetPosition(denc);
//        LEFTFRONT.setTargetPosition(denc);
//
//        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        telemetry.addData("Mode", "running");
//        telemetry.update();
//
//        RIGHTFRONT.setPower(power);
//        LEFTFRONT.setPower(power);
//        RIGHTBACK.setPower(power);
//        LEFTBACK.setPower(power);
//
//        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//        {
//            telemetry.addData("encoder-back-left", LEFTBACK.getCurrentPosition() + "  busy=" + LEFTBACK.isBusy());
//            telemetry.addData("encoder-forward-left", LEFTFRONT.getCurrentPosition() + "  busy=" + LEFTFRONT.isBusy());
//            telemetry.addData("encoder-back-right", RIGHTBACK.getCurrentPosition() + "  busy=" + RIGHTBACK.isBusy());
//            telemetry.addData("encoder-forward-right", RIGHTFRONT.getCurrentPosition() + "  busy=" + RIGHTFRONT.isBusy());
//
//            telemetry.update();
//            idle();
//        }
//
//        stopEverything();
//    }
//
//    private void CrabForDistance(double power, double revolutions) {
//        //positive revolutions = left
//        //negative revolutions = right
//        int denc = (int)Math.round(revolutions * encRotation);
//
//        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
//        LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
//        RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
//        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        RIGHTFRONT.setTargetPosition(denc);
//        LEFTBACK.setTargetPosition(denc);
//        RIGHTBACK.setTargetPosition(denc);
//        LEFTFRONT.setTargetPosition(denc);
//
//        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        telemetry.addData("Mode", "running");
//        telemetry.update();
//
//        RIGHTFRONT.setPower(power);
//        LEFTFRONT.setPower(power);
//        RIGHTBACK.setPower(power);
//        LEFTBACK.setPower(power);
//
//        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//        {
//            telemetry.addData("encoder-back-left", LEFTBACK.getCurrentPosition() + "  busy=" + LEFTBACK.isBusy());
//            telemetry.addData("encoder-forward-left", LEFTFRONT.getCurrentPosition() + "  busy=" + LEFTFRONT.isBusy());
//            telemetry.addData("encoder-back-right", RIGHTBACK.getCurrentPosition() + "  busy=" + RIGHTBACK.isBusy());
//            telemetry.addData("encoder-forward-right", RIGHTFRONT.getCurrentPosition() + "  busy=" + RIGHTFRONT.isBusy());
//
//            telemetry.update();
//            idle();
//        }
//
//        stopEverything();
//    }
//
//    private void resetEncoders() {
//        //LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//}
