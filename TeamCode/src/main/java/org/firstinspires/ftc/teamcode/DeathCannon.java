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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DEATH CANNON", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class DeathCannon extends LinearOpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totTime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor deathcannon = null;

    private double turnPower = .6; //for 90 degrees
    private double turnTime = .5; //for 90 degrees

    @Override
    public void runOpMode () throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialize hardware variables. Same as configured on phone
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        deathcannon = hardwareMap.dcMotor.get("deathcannon");
        //Set the drive motor directions
        //right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        totTime.reset();

        back(.3);
        tele("forward", 1);
        stopMoving();
        tele("delay", 1);
        //BLUE LEFT / RED RIGHT
        turn45("right");

        shoots("go");
        tele("deathcannon", .8);
        shoots("stop");

        telemetry.addData("Time","Total time: " + totTime.seconds());
        tele("total time", 1);
    }

    //TELEMETRY METHOD
    public void tele(String event, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(event, "S Elapsed " + runtime.seconds());
            telemetry.addData("Deathcannon", deathcannon.getPower());
            telemetry.addData("Right ", left.getPower());
            telemetry.addData("Left ", right.getPower());
            telemetry.update();
        }
    }
    //TELEMETRY METHOD

    //GUN METHOD
    public void shoots(String gostop){
        if(opModeIsActive() && gostop.equals("go")) {
            deathcannon.setPower(-1);
        }
        if (opModeIsActive() && gostop.equals("stop")){
            deathcannon.setPower(0);
        }
    }
    //GUN METHOD

    //TURN METHODS
    public void turn90(String LoR){
        if(LoR.equals("right") && opModeIsActive()){
            left.setPower(-turnPower);
            right.setPower(turnPower);
            tele("turn right", turnTime);
        }
        if(LoR.equals("left")&& opModeIsActive()){
            left.setPower(turnPower);
            right.setPower(-turnPower);
            tele("turn left", turnTime);
        }
        stopMoving();
        tele("delay", 1);
    }
    public void turn45(String LoR){
        if(LoR.equals("right") && opModeIsActive()){
            left.setPower(-turnPower);
            right.setPower(turnPower);
            tele("turn right", turnTime/2);
        }
        if(LoR.equals("left")&& opModeIsActive()){
            left.setPower(turnPower);
            right.setPower(-turnPower);
            tele("turn left", turnTime/2);
        }
        stopMoving();
        tele("delay", 1);
    }
    //TURN METHOD

    //FORWARDS BACKWARDS STOP METHODS
    public void forward(double power) {
        left.setPower(power);
        right.setPower(power);
    }
    public void back(double power) {
        left.setPower(-power);
        right.setPower(-power);
    }
    public void stopMoving() {
        left.setPower(0);
        right.setPower(0);
    }
    //FORWARDS BACKWARDS STOP METHODS
}
