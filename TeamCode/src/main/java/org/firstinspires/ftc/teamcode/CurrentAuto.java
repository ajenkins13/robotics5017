//TODO(sachi): Rename this...

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

@Autonomous(name = "Current Auto", group = "") //name of the file

public class CurrentAuto extends LinearOpMode { //creating public class, extension of linear opmode

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

            //TEST ALL FUNCTIONS
            // ForwardForDistance(0.5, 5);
            // ForwardForDistance(0.5, -5);

            // sleep(2000);

            // TurnForDistance(0.5, 1);
            // TurnForDistance(0.5, -1);

            // sleep(2000);

            // CrabForDistance(0.5,1);
            // CrabForDistance(0.5,-1);

            //reach the launch line

            SHOOTER.setDirection(DcMotorSimple.Direction.FORWARD);
            SHOOTER.setPower(0.52); //maybe max?

            CrabForDistance(0.2, 1);
            sleep(500);
            ForwardForDistance(0.5, -1);
            sleep(1000);
            ForwardForDistance(0.5, 4.7);
            sleep(1000);
            TurnForDistance(0.5,-0.18);

            //shoot 3 rings at the high goal

            sleep(3000);
            FLICKER.setPosition(.2);
            sleep(500);
            FLICKER.setPosition(1);

            sleep(3000);
            FLICKER.setPosition(.2);
            sleep(500);
            FLICKER.setPosition(1);

            sleep(3000);
            FLICKER.setPosition(.2);
            sleep(500);
            FLICKER.setPosition(1);
            sleep(500);
            FLICKER.setPosition(.2);
            sleep(500);
            FLICKER.setPosition(1);
            sleep(1000);

            // WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
            // WOBBLE.setPower(1); //position = placeholder --> replace later after testing
            // sleep(500);
            // WOBBLE.setPower(0.5); //position = placeholder --> replace later after testing
            // sleep(200);
            // WOBBLE.setPower(0.4); //position = placeholder --> replace later after testing
            // sleep(200);
            // WOBBLE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // WOBBLE.setPower(0);

            ForwardForDistance(0.5, 1.5);

            //move over to the target square
            //ForwardForDistance(0.5, -1.5);
            //sleep(500);
            //crab left to the target zone
            //CrabForDistance(1, 1);
            //SHOOTER.setPower(0);

            //drop the wobble goal in the launch line target zone
            // WOBBLE.setDirection(DcMotorSimple.Direction.REVERSE);
            // WOBBLE.setPower(1); //position = placeholder --> replace later after testing
            // sleep(500);
            // WOBBLE.setPower(0.5); //position = placeholder --> replace later after testing
            // sleep(200);
            // WOBBLE.setPower(0.4); //position = placeholder --> replace later after testing
            // sleep(200);
            // WOBBLE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // WOBBLE.setPower(0);

            //sleep(2000);
            //WOBBLEBLOCK.setPosition(0);
            //sleep(1000);

            //ForwardForDistance(0.3,1);

            //park on the launch line (not touching the wobble goal)
            //crab right
            //CrabForDistance(0.5,-1);

            //ForwardForDistance(0.5, -1);

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
