        /*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
//red
public class Red extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor gun = null;
    private Servo button;
    private ColorSensor color;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");
        gun = hardwareMap.dcMotor.get("gun");
        button = hardwareMap.servo.get("button");
        color = hardwareMap.colorSensor.get("color");
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        button.setPosition(.87);

        gunShoots(.5);
        Telemetry("Shoots", 2);
        gunShoots(0);

        turnLeft(.3);
        Telemetry("turn", .5);
        driveForward(.8);
        Telemetry("forward", .5);
        //exact same as before turn
        turnRight(.3);
        Telemetry("turn", .5);

        driveForward(.8);
        if(color.red() > 2){
            driveForward(.3);
            if(color.red() == color.blue()){
                driveBack(.3);
                Telemetry("back", .1);
                stopMoving();

                button.setPosition(1);
                button.setPosition(.87);

                driveForward(.8);
                Telemetry("forward", .5);
            }
        }
        else if(color.blue() > 2){
            driveForward(.3);
            if(color.red() == color.blue()){
                driveForward(.3);
                Telemetry("forward", .08);
                stopMoving();
                button.setPosition(1);
                button.setPosition(.87);

                driveForward(.8);
                Telemetry("forward", .5);
            }
        }

        driveForward(.8);
        if(color.red() > 2){
            driveForward(.3);
            if(color.red() == color.blue()){
                driveBack(.3);
                Telemetry("back", .1);
                stopMoving();

                button.setPosition(1);
                button.setPosition(.87);
            }
        }
        else if(color.blue() > 2){
            driveForward(.3);
            if(color.red() == color.blue()){
                driveForward(.3);
                Telemetry("forward", .08);
                stopMoving();

                button.setPosition(1);
                button.setPosition(.87);
            }
        }


    }

    public void Telemetry(String event, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(event, "S Elapsed " + runtime.seconds());
            telemetry.addData("Red ", color.red());
            telemetry.addData("Blue ", color.blue());
            telemetry.addData("Right ", leftMotor.getPower());
            telemetry.addData("Left ", rightMotor.getPower());
            telemetry.addData("Gun ", gun.getPower());
            telemetry.update();
        }
    }

    public void driveForward(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void driveBack(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
    }

    public void stopMoving(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        gun.setPower(0);
    }

    public void turn(double powerRight, double powerLeft) {
        leftMotor.setPower(powerLeft);
        rightMotor.setPower(powerRight);
    }

    public  void turnLeft(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public  void turnRight(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    public void gunShoots(double gunPower) {
        gun.setPower(-gunPower);
    }

}
