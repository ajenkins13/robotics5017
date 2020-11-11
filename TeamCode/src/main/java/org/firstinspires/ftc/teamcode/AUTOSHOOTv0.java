//comment out attachments since this is for this year

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

@Autonomous(name = "AUTOSHOOTv0", group = "") //name of the file

public class AUTOSHOOTv0 extends LinearOpMode { //creating public class, extension of linear opmode

    //creating motors, touch sensors, and servos
    private DcMotor RIGHTFRONT;
    private DcMotor RIGHTBACK;
    private DcMotor LEFTFRONT;
    private DcMotor LEFTBACK;
    private DcMotor SHOOTER;
    private Servo FLICKER;

    @Override
    public void runOpMode() {
        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
        SHOOTER = hardwareMap.dcMotor.get("SHOOTER");
        //WOBBLE = hardwareMap.dcMotor.get("WOBBLE");
        //INTAKE = hardwareMap.dcMotor.get("INTAKE");

        FLICKER = hardwareMap.servo.get("FLICKER");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            BackwardForTime(1,500);

            SHOOTER.setDirection(DcMotorSimple.Direction.REVERSE);
            SHOOTER.setPower(1); //maybe max?
            sleep(500);
            FLICKER.setPosition(.2);
            sleep(500);
            FLICKER.setPosition(1);

            sleep(1000);
            FLICKER.setPosition(.2);
            sleep(500);
            FLICKER.setPosition(1);

            sleep(1000);
            FLICKER.setPosition(.2);
            sleep(500);
            FLICKER.setPosition(1);

        }
    }

    private void stopEverything() {
        LEFTFRONT.setPower(0);
        RIGHTFRONT.setPower(0);
        LEFTBACK.setPower(0);
        RIGHTBACK.setPower(0);
    }

    private void ForwardForTime(double power, long time) { //FIXED
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);
        sleep(time);
    }

    private void BackwardForTime(double power, long time) { //FIXED
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);
        sleep(time);
    }

    private void CrabForTime(double power, long time) {
        //positive power = right
        //negative power = left
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(-power);
        LEFTBACK.setPower(-power);
        sleep(time);
    }

    private void TurnLForTime(double power, long time) {
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTBACK.setDirection(DcMotorSimple.Direction.FORWARD);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(-power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(-power);
        sleep(time);
    }

    private void TurnRForTime(double power, long time) {
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(-power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(-power);
        sleep(time);
    }

    private void resetEncoders() {
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
