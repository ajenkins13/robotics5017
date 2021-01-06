//this is edited
//i saw these edits
package org.firstinspires.ftc.teamcode; //importing OUR package

//importing OpModes (linear and teleOp) and importing hardware (motors, sensors, servos)
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TELEOPROBOT2", group = "") //name of file


public class TELEOPROBOT2 extends LinearOpMode { //declaring class for whole program

    //initialize instance variables
    private DcMotor LEFTFRONT; //2:0
    private DcMotor LEFTBACK; //2:1
    private DcMotor RIGHTFRONT; //1:0
    private DcMotor RIGHTBACK; //1:1
    private DcMotor SHOOTER; //1:1
    private DcMotor WOBBLE;
    private DcMotor INTAKE;
    private Servo WOBBLEBLOCK;
    private CRServo POPUP;
    private Servo FLICKER;


    @Override
    public void runOpMode() {
        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
        SHOOTER = hardwareMap.dcMotor.get("SHOOTER");
        WOBBLE = hardwareMap.dcMotor.get("WOBBLE");
        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        WOBBLEBLOCK = hardwareMap.servo.get("WOBBLEBLOCK");
        POPUP = hardwareMap.crservo.get("POPUP");
        FLICKER = hardwareMap.servo.get("FLICKER");

        sleep(1000);

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) { //looking for values, waiting for controller to send values

                // ********** DRIVE TRAIN **********

                if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1) { //if left stick or right stick is pushed a significant amount
                    // Forward/Reverse Drive
                    RIGHTFRONT.setPower(-gamepad1.right_stick_y);
                    RIGHTBACK.setPower(-gamepad1.right_stick_y);
                    LEFTFRONT.setPower(gamepad1.left_stick_y);
                    LEFTBACK.setPower(gamepad1.left_stick_y);
                }

                else if (Math.abs(gamepad1.right_trigger) != 0) {
                    // Right Crab
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    RIGHTFRONT.setPower(-gamepad1.right_trigger);
                    LEFTBACK.setPower(gamepad1.right_trigger);
                    RIGHTBACK.setPower(gamepad1.right_trigger);
                    LEFTFRONT.setPower(-gamepad1.right_trigger);
                }

                else if (Math.abs(gamepad1.left_trigger) != 0) {
                    // Left Crab
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    RIGHTFRONT.setPower(gamepad1.left_trigger);
                    LEFTBACK.setPower(-gamepad1.left_trigger);
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    RIGHTBACK.setPower(-gamepad1.left_trigger);
                    LEFTFRONT.setPower(gamepad1.left_trigger);
                }

                else {
                    RIGHTFRONT.setPower(0); //if not triggered, make sure motors donÃ¢â‚¬â„¢t move/power=0
                    LEFTFRONT.setPower(0);
                    RIGHTBACK.setPower(0);
                    LEFTBACK.setPower(0);
                }


                // ********** ATTACHMENTS **********

                if (gamepad1.y) { //shoot for highest goal
                    SHOOTER.setDirection(DcMotorSimple.Direction.REVERSE);
                    SHOOTER.setPower(1); //maybe max?
                    sleep(500);
                    FLICKER.setPosition(.2);
                    sleep(500);
                    FLICKER.setPosition(1);
                }

                else if (gamepad1.x) {  // resets all motors (cancels everything).
                    SHOOTER.setPower(0);
                    FLICKER.setPosition(0.5);
                    INTAKE.setPower(0);
                }

                else if (gamepad1.b) {  // resets all motors (cancels everything).
                    INTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
                    INTAKE.setPower(1);
                }

                else if (gamepad1.a) {
                    //puts shooter at medium power to hit power shot target
                    SHOOTER.setDirection(DcMotorSimple.Direction.REVERSE);
                    SHOOTER.setPower(0.7); //maybe max?
                    sleep(500);
                    FLICKER.setPosition(.2);
                    sleep(500);
                    FLICKER.setPosition(1);
                }

                else if (gamepad2.y) { //shoot for mid goal/powershot
                    WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
                    WOBBLE.setPower(.5); //maybe max?
                    sleep(100);
                    WOBBLE.setPower(0); //maybe max?
                    WOBBLE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

                else if (gamepad2.a) {
                    //move wobbleblock to open
                    WOBBLEBLOCK.setPosition(0);
                }

                else if (gamepad2.x) {
                    //move wobbleblock to close
                    WOBBLEBLOCK.setPosition(1);
                }

                else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                    //controls wheels that pop up rings to reach the ramp
                    POPUP.setPower(gamepad1.left_stick_y);
                }
                //LEFT BUMPER TO RUN INTAKE BACKWARDS?
            }
        }
    }
}