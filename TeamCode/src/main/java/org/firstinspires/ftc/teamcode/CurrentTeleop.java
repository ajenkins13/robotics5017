//this is edited
//i saw these edits
//sounds good
package org.firstinspires.ftc.teamcode; //importing OUR package

//importing OpModes (linear and teleOp) and importing hardware (motors, sensors, servos)
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.reflect.Array;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Current Teleop", group = "") //name of file


public class CurrentTeleop extends LinearOpMode { //declaring class for whole program

    //initialize instance variables
    private DcMotor LEFTFRONT; //2:0
    private DcMotor LEFTBACK; //2:1
    private DcMotor RIGHTFRONT; //1:0
    private DcMotor RIGHTBACK; //1:1
    private DcMotorEx SHOOTER; //1:1
    private DcMotor WOBBLE;
    private DcMotor INTAKE;
    private Servo WOBBLEBLOCK;
    private Servo FLICKER;
    private Servo KICKOUT;
    private Servo MRFIXIT;
    private Servo SLICER;

    @Override
    public void runOpMode() {
        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
        SHOOTER = (DcMotorEx)(hardwareMap.dcMotor.get("SHOOTER"));
        WOBBLE = hardwareMap.dcMotor.get("WOBBLE");
        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        WOBBLEBLOCK = hardwareMap.servo.get("WOBBLEBLOCK");
        FLICKER = hardwareMap.servo.get("FLICKER");
        KICKOUT = hardwareMap.servo.get("KICKOUT");
        MRFIXIT = hardwareMap.servo.get("MRFIXIT");
        SLICER = hardwareMap.servo.get("SLICER");

        sleep(1000);

        waitForStart();

        if (opModeIsActive()) {

            MRFIXIT.setPosition(0); // on init, to release the fixer from auto (left is out/0).

            while (opModeIsActive()) { //looking for values, waiting for controller to send values

                // ********** DRIVE TRAIN **********

                if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1) { //if left stick or right stick is pushed a significant amount
                    // Forward/Reverse Drive
                    RIGHTFRONT.setPower(-gamepad1.right_stick_y);
                    RIGHTBACK.setPower(-gamepad1.right_stick_y);
                    LEFTFRONT.setPower(-gamepad1.left_stick_y);
                    LEFTBACK.setPower(-gamepad1.left_stick_y);
                }

                else if (Math.abs(gamepad1.right_trigger) != 0) {
                    // Right Crab
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    RIGHTFRONT.setPower(-gamepad1.right_trigger);
                    LEFTBACK.setPower(-gamepad1.right_trigger);
                    RIGHTBACK.setPower(gamepad1.right_trigger);
                    LEFTFRONT.setPower(gamepad1.right_trigger);
                    //telemetry.addData("hi", "");
                   // telemetry.update();
                }

                else if (Math.abs(gamepad1.left_trigger) != 0) {
                    // Left Crab
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    RIGHTFRONT.setPower(gamepad1.left_trigger);
                    LEFTBACK.setPower(gamepad1.left_trigger);
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    RIGHTBACK.setPower(-gamepad1.left_trigger);
                    LEFTFRONT.setPower(-gamepad1.left_trigger);
                }

                else {
                    RIGHTFRONT.setPower(0); //if not triggered, make sure motors don??????????????????t move/power=0
                    LEFTFRONT.setPower(0);
                    RIGHTBACK.setPower(0);
                    LEFTBACK.setPower(0);
                }


                // ********** ATTACHMENTS **********

                if (gamepad1.y) { //shoot for highest goal

                    sleep(500);
                    FLICKER.setPosition(0);
                    sleep(500);
                    FLICKER.setPosition(1);
                }

                if (gamepad1.x) {  // resets all motors (cancels everything).
                    SHOOTER.setPower(0);

                }

                if (gamepad1.b) {
                    //puts shooter at medium power to hit power shot target
                    SHOOTER.setDirection(DcMotorSimple.Direction.FORWARD);
                    SHOOTER.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SHOOTER.setVelocity(1885);   // Ticks per second.
                   // SHOOTER.setPower(.57);//95 is power shot number
                   // sleep(500);
                }


                if (gamepad1.a) {
                    //flicker

                    FLICKER.setPosition(.2);
                    sleep(500);
                    FLICKER.setPosition(.7);
                    MRFIXIT.setPosition(.5);//trap it
                    sleep( 500);
                    MRFIXIT.setPosition(0);//trap it
                }

                if (gamepad1.right_bumper == true) {
                    SLICER.setPosition(.5);
                }

                if (gamepad2.y) {
                    //intake goes in
                    INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
                    INTAKE.setPower(1);
                }


                if (gamepad2.a) {
                    //intake goes out
                    INTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
                    INTAKE.setPower(1);
                }

                if (gamepad2.b){
                    SHOOTER.setDirection(DcMotorSimple.Direction.FORWARD);
                    SHOOTER.setPower(.45);//95 is power shot number
                    sleep(500);
                }

                if (gamepad2.x) {
                    //stop intake
                    INTAKE.setPower(0);
                }

                if (gamepad2.right_stick_y > 0.1) {

                    //sets wobble out, reverse means going toward robot

                    WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
                    WOBBLE.setPower((gamepad2.right_stick_y));

                }
                if ((gamepad2.right_stick_y) < -0.1) {
                    //brings wobble in, forward is away from the robot
                    WOBBLE.setDirection(DcMotorSimple.Direction.FORWARD);
                    WOBBLE.setPower(Math.abs(gamepad2.right_stick_y));

                }

                if (gamepad2.right_bumper == true) {

                    sleep(500);
                    WOBBLEBLOCK.setPosition(0);
                    KICKOUT.setPosition(1);
                    sleep(2000);
                    KICKOUT.setPosition(0);

                } if ((gamepad2.left_bumper) == true) {

                    sleep(500);
                    WOBBLEBLOCK.setPosition(1);

                }

                if (gamepad2.left_trigger > .1) {
                    MRFIXIT.setPosition(.5);

                }
                if (gamepad2.right_trigger > .1) {
                    MRFIXIT.setPosition(0);

                }

            }
        }
    }


}