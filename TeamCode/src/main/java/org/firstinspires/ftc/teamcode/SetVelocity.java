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

@TeleOp(name = "Set Velocity", group = "") //name of file

/* Example of how to set the velocity of a motor (for the flywheel) to a constant velocity
   rather than a power value.
 */
public class SetVelocity extends LinearOpMode { //declaring class for whole program

    private DcMotorEx SHOOTER; //1:1

    @Override
    public void runOpMode() {

        SHOOTER = (DcMotorEx)(hardwareMap.dcMotor.get("SHOOTER"));

        sleep(1000);

        waitForStart();

        if (opModeIsActive()) {

            telemetry.addData("Starting velocity:", "" + SHOOTER.getVelocity());
            SHOOTER.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SHOOTER.setVelocity(500);   // Ticks per second.
            // We don't want this one: SHOOTER.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);
        }
    }


}