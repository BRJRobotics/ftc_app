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
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RED Beacons", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
/*
 * Red Team
 *  Red Team
 *   Red Team
 *    Red Team
 *     Red Team
 */
public class RedBeacon extends LinearOpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totTime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private Servo gear;
    private TouchSensor button;
    private ColorSensor color;

    private double speedOfButton = .1;
    private double timeOfButton = .45;
    private double turnPower = .6; //for 90 degrees
    private double turnTime = .5; //for 90 degrees
    private double minServo = Servo.MIN_POSITION;
    private double maxServo = Servo.MAX_POSITION * 7 / 10;

    @Override
    public void runOpMode () throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialize hardware variables. Same as configured on phone
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        button = hardwareMap.touchSensor.get("button");
        gear = hardwareMap.servo.get("gear");
        color = hardwareMap.colorSensor.get("color");
        //Set the drive motor directions
        //right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        totTime.reset();

        //path
        back(.3);
        tele("path", 1.6);
        stopMoving();
        tele("delay",1);
        // BLUE RIGHT / RED LEFT
        turn90("left");
        //
        back(.3);
        tele("path",2.45);
        stopMoving();
        tele("delay",1);
        //
        turn90("left");
        //path

        //beacons
        beacon();
        back(.05);
        tele("first color", 5);
        beacon();
        //beacons

        telemetry.addData("Time","Total time: " + totTime.seconds());
        tele("total time", 1);
    }

            /*Order of methods: Telemetry, firstColor, colorsEqual,
                beacon, buttonExtends, turn90, forwards, back, stopMoving*/

    //TELEMETRY METHOD
    public void tele(String event, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(event, "S Elapsed " + runtime.seconds());
            telemetry.addData("Red ", color.red());
            telemetry.addData("Blue ", color.blue());
            telemetry.addData("Gear ", gear.getPosition());
            telemetry.addData("Button ", button.getValue());
            telemetry.addData("Right ", left.getPower());
            telemetry.addData("Left ", right.getPower());
            telemetry.update();
        }
    }
    //TELEMETRY METHOD

    //MOVE TILL FIRST COLOR METHOD
    public void firstColor(){
        runtime.reset();
        while (opModeIsActive() && (color.red() <= 1 && color.blue() <= 1)) {
            //BLUE BACK RED FORWARD
            forward(.1);
            telemetry.addData("first color", "S Elapsed " + runtime.seconds());
            telemetry.addData("Red ", color.red());
            telemetry.addData("Blue ", color.blue());
            telemetry.addData("Gear ", gear.getPosition());
            telemetry.addData("Button ", button.getValue());
            telemetry.addData("Right ", left.getPower());
            telemetry.addData("Left ", right.getPower());
            telemetry.update();
        }
    }
    //MOVE TILL FIRST COLOR METHOD

    //MOVE TILL MIDDLE METHOD
    public void colorsEqual(){
        runtime.reset();
        while (opModeIsActive() && color.red() != color.blue()  && color.red() != 1 && color.blue() !=1) {
            //BLUE BACK RED FORWARD
            forward(.05);
            telemetry.addData("checking", "S Elapsed " + runtime.seconds());
            telemetry.addData("Red ", color.red());
            telemetry.addData("Blue ", color.blue());
            telemetry.addData("Gear ", gear.getPosition());
            telemetry.addData("Button ", button.getValue());
            telemetry.addData("Right ", left.getPower());
            telemetry.addData("Left ", right.getPower());
            telemetry.update();
        }
        stopMoving();
    }
    //MOVE TILL MIDDLE METHOD

    //BEACON METHOD
    public void beacon(){
        firstColor();
        if (color.red() > 1 && opModeIsActive()) {
            colorsEqual();
            back(speedOfButton);
            tele("calibrate", timeOfButton);
            stopMoving();
            buttonExtends("Sees RED");
        } else if (color.blue() > 1 && opModeIsActive()) {
            colorsEqual();
            forward(speedOfButton);
            tele("calibrate", timeOfButton);
            stopMoving();
            buttonExtends("Sees BLUE");
        }
        gear.setPosition(maxServo);
        tele("Return", 1);
    }
    //BEACON METHOD

    //BUTTON EXTENDS METHOD
    public void buttonExtends(String colorDetected){
        for (double position = maxServo;opModeIsActive() && position > minServo;position -= .01) {
            gear.setPosition(position);
            tele(colorDetected, .01);
            if (button.isPressed() && opModeIsActive()) {
                break;
            }
        }
    }
    //BUTTON EXTENDS METHOD

    //TURN METHOD
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

            /*Order of methods: Telemetry, firstColor, colorsEqual,
                beacon, buttonExtends, turn90, forwards, back, stopMoving*/
}
