package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="Center Park Red", group="Linear Opmode")
public class CenterParkTest extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left;
    private DcMotor right;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Initialize the hardware variables. Same as phone configuration
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        //Set the drive motor directions
        right.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        back(.8);
        Telemetry("Foward", .98);

        turnRight(0.5);
        Telemetry("Turn", .95);

        back(-0.5);
        Telemetry("Ramp", .3);
    }

    public void Telemetry(String event, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(event, "S Elapsed " + runtime.seconds());
            telemetry.addData("Right ", left.getPower());
            telemetry.addData("Left ", right.getPower());
            telemetry.update();
        }
    }

    public void forward(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void back(double power) {
        left.setPower(-power);
        right.setPower(-power);
    }

    public void stopMoving(){
        left.setPower(0);
        right.setPower(0);
    }

    public void turn(double powerRight, double powerLeft) {
        left.setPower(powerLeft);
        right.setPower(powerRight);
    }

    public  void turnLeft(double power) {
        turn(power,-power);
    }

    public  void turnRight(double power) {
        turn(-power, power);
    }
}
