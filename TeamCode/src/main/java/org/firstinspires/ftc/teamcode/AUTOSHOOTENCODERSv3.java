//comment out attachments since this is for this year
//FORWARD means REVERSE for the left wheels

package org.firstinspires.ftc.teamcode; //importing OUR package

import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //auto this time, importing
//importing OpModes (linear and teleOp) and importing hardware (motors, sensors, servos)
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//importing servos, motors, touch sensors
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "currentProgram", group = "") //name of the file

public class AUTOSHOOTENCODERSv3 extends LinearOpMode { //creating public class, extension of linear opmode

    //creating motors, touch sensors, and servos
    private DcMotor RIGHTFRONT;
    private DcMotor RIGHTBACK;
    private DcMotor LEFTFRONT;
    private DcMotor LEFTBACK;
    private DcMotor SHOOTER;
    private Servo FLICKER;
    private DcMotor WOBBLE;
    private Servo WOBBLEBLOCK;
    private DcMotor INTAKE;

    final double encRotation = 537.6;

    @Override
    public void runOpMode() {
        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
        SHOOTER = hardwareMap.dcMotor.get("SHOOTER");
        WOBBLE = hardwareMap.dcMotor.get("WOBBLE");
        WOBBLEBLOCK = hardwareMap.servo.get("WOBBLEBLOCK");
        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        FLICKER = hardwareMap.servo.get("FLICKER");

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            //reach the launch line
            CrabForDistance(0.5, 1);
            sleep(500);
            ForwardForDistance(0.5, -4);
            sleep(1000);

            //shoot 3 rings at the high goal
//            SHOOTER.setDirection(DcMotorSimple.Direction.REVERSE);
//            SHOOTER.setPower(1); //maybe max?
//            sleep(500);
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

            //move over to the target square
            //move forward
            ForwardForDistance(0.5, -1.5);
            sleep(500);
            //crab left to the target zone
            CrabForDistance(0.5, 2);

            //drop the wobble goal in the launch line target zone
//            WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
//            WOBBLE.setPower(0.4); //position = placeholder --> replace later after testing
//            sleep(200);
//            WOBBLE.setPower(0);

            //park on the launch line (not touching the wobble goal)
            //crab right
            CrabForDistance(0.5,-1);

//            telemetry.addData("test", "servo position: " + WOBBLEBLOCK.getPosition());
//            WOBBLEBLOCK.setPosition(0);
//            //telemetry.update();
//            telemetry.addData("test", "servo position: " + WOBBLEBLOCK.getPosition());
//            WOBBLEBLOCK.setPosition(1);
//            //telemetry.update();
//            telemetry.addData("test", "servo position: " + WOBBLEBLOCK.getPosition());
//            WOBBLEBLOCK.setPosition(0);
//            telemetry.update();
//            sleep(10000);
            //idle();
        }
    }

    private void stopEverything() {
        LEFTFRONT.setPower(0);
        RIGHTFRONT.setPower(0);
        LEFTBACK.setPower(0);
        RIGHTBACK.setPower(0);
    }

    private void ForwardForDistance(double power, double revolutions) {
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
            telemetry.addData("encoder-back-left", LEFTBACK.getCurrentPosition() + "  busy=" + LEFTBACK.isBusy());
            telemetry.addData("encoder-forward-left", LEFTFRONT.getCurrentPosition() + "  busy=" + LEFTFRONT.isBusy());
            telemetry.addData("encoder-back-right", RIGHTBACK.getCurrentPosition() + "  busy=" + RIGHTBACK.isBusy());
            telemetry.addData("encoder-forward-right", RIGHTFRONT.getCurrentPosition() + "  busy=" + RIGHTFRONT.isBusy());

            telemetry.update();
            idle();
        }

        stopEverything();
    }

    private void TurnForDistance(double power, double revolutions) {
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

        telemetry.addData("Mode", "running");
        telemetry.update();

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);

        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", LEFTBACK.getCurrentPosition() + "  busy=" + LEFTBACK.isBusy());
            telemetry.addData("encoder-forward-left", LEFTFRONT.getCurrentPosition() + "  busy=" + LEFTFRONT.isBusy());
            telemetry.addData("encoder-back-right", RIGHTBACK.getCurrentPosition() + "  busy=" + RIGHTBACK.isBusy());
            telemetry.addData("encoder-forward-right", RIGHTFRONT.getCurrentPosition() + "  busy=" + RIGHTFRONT.isBusy());

            telemetry.update();
            idle();
        }

        stopEverything();
    }

    private void CrabForDistance(double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
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

        telemetry.addData("Mode", "running");
        telemetry.update();

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);

        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", LEFTBACK.getCurrentPosition() + "  busy=" + LEFTBACK.isBusy());
            telemetry.addData("encoder-forward-left", LEFTFRONT.getCurrentPosition() + "  busy=" + LEFTFRONT.isBusy());
            telemetry.addData("encoder-back-right", RIGHTBACK.getCurrentPosition() + "  busy=" + RIGHTBACK.isBusy());
            telemetry.addData("encoder-forward-right", RIGHTFRONT.getCurrentPosition() + "  busy=" + RIGHTFRONT.isBusy());

            telemetry.update();
            idle();
        }

        stopEverything();
    }

    private void resetEncoders() {
        //LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
