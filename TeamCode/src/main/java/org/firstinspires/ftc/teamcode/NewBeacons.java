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

        /*
            GOALS
            Do all of the timings
            Pin servo (note probably needs more timing things)
            Understand the Reversing of wheels, in blue just make negative values bc other will confuse
         */

        @Autonomous(name="NewBlue", group="Linear Opmode")  // @Autonomous(...) is the other common choice
        @Disabled

        /*
        * Blue team
        * Blue team
        * Blue team
        * Blue team
        * Blue team
        * Blue team
        * Blue team
         */
        public class NewBeacons extends LinearOpMode {

            /* Declare OpMode members. */
            private ElapsedTime runtime = new ElapsedTime();
            private DcMotor left;
            private DcMotor right;

            private DcMotor gun;
            private Servo feeder;

            private Servo gear;
            private TouchSensor button;
            private ColorSensor color;

            @Override
            public void runOpMode() {
                telemetry.addData("Status", "Initialized");
                telemetry.update();

                /* eg: Initialize the hardware variables. Note that the strings used here as parameters
                 * to 'get' must correspond to the names assigned during the robot configuration
                 * step (using the FTC Robot Controller app on the phone).
                 */
                left = hardwareMap.dcMotor.get("left");
                right = hardwareMap.dcMotor.get("right");

                feeder = hardwareMap.servo.get("feeder");
                gun = hardwareMap.dcMotor.get("gun");

                button = hardwareMap.touchSensor.get("button");
                gear = hardwareMap.servo.get("gearServo");
                color = hardwareMap.colorSensor.get("color");
                // eg: Set the drive motor directions:
                // "Reverse" the motor that runs backwards when connected directly to the battery
                // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

                // Wait for the game to start (driver presses PLAY)
                waitForStart();


                gear.setPosition(0);

                //might change if go towards goal forward, turn 90, turn opposite way 90
                driveForward(.5);//Just a bit to shoot gun
                Telemetry("Forward", .5);

                //aim it
                gunShoots(.5);
                Telemetry("Shoots", 2);
                gunShoots(0);

                driveBack(.5);
                Telemetry("Back", .5);
                turnLeft(.3);//Turn 90
                Telemetry("turn", .5);
                driveForward(.8);//Line up w/ beacons
                Telemetry("forward", .5);
                //exact same as before turn - to be parallel with beacon
                turnRight(.3);//Turn 90
                Telemetry("turn", .5);

                //Do twice for two beacons
                for (int twi = 0; twi < 2; twi++) {
                    runtime.reset();
                    while (color.red() <= 1 && color.blue() <= 1) {
                        driveForward(.5);
                        telemetry.addData("checking", "S Elapsed " + runtime.seconds());
                        telemetry.addData("Red ", color.red());
                        telemetry.addData("Blue ", color.blue());
                        telemetry.addData("Gear ", gear.getPosition());
                        telemetry.addData("Button ", button.getValue());
                        telemetry.addData("Right ", left.getPower());
                        telemetry.addData("Left ", right.getPower());
                        telemetry.update();
                    }
                    if (color.red() > 1) {
                        runtime.reset();
                        while (color.red() != color.blue()) {
                            driveForward(.3);
                            telemetry.addData("checking", "S Elapsed " + runtime.seconds());
                            telemetry.addData("Red ", color.red());
                            telemetry.addData("Blue ", color.blue());
                            telemetry.addData("Gear ", gear.getPosition());
                            telemetry.addData("Button ", button.getValue());
                            telemetry.addData("Right ", left.getPower());
                            telemetry.addData("Left ", right.getPower());
                            telemetry.update();
                        }
                        driveForward(0);
                        Telemetry("Just a pause",.2);

                        driveForward(.2);
                        Telemetry("Align Button",.2);

                        for (double position = Servo.MIN_POSITION; position < Servo.MAX_POSITION * 3 / 5; position += .01) {
                            gear.setPosition(position);
                            Telemetry("RED", .01);
                            if (button.isPressed()) {
                                break;
                            }
                        }
                    } else if (color.blue() > 1) {
                        runtime.reset();
                        while (color.red() != color.blue()) {
                            driveForward(.3);
                            telemetry.addData("checking", "S Elapsed " + runtime.seconds());
                            telemetry.addData("Red ", color.red());
                            telemetry.addData("Blue ", color.blue());
                            telemetry.addData("Gear ", gear.getPosition());
                            telemetry.addData("Button ", button.getValue());
                            telemetry.addData("Right ", left.getPower());
                            telemetry.addData("Left ", right.getPower());
                            telemetry.update();
                        }
                        driveForward(0);

                        driveBack(.2);
                        Telemetry("Align Button",.2);

                        for (double position = Servo.MIN_POSITION; position < Servo.MAX_POSITION * 3 / 5; position += .01) {
                            gear.setPosition(position);
                            Telemetry("BLUE", .01);
                            if (button.isPressed()) {
                                break;
                            }
                        }
                    }
                    gear.setPosition(Servo.MIN_POSITION);
                    Telemetry("Return", 1);
                    //So it deosnt read the current bacon
                    driveForward(1);
                    Telemetry("Skip", 0.5);
                    driveForward(0);
                }
            }

            public void Telemetry(String event, double time) {
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < time)) {
                    telemetry.addData(event, "S Elapsed " + runtime.seconds());

                    telemetry.addData("Gun ", gun.getPower());
                    telemetry.addData("Feeder ", feeder.getPosition());

                    telemetry.addData("Red ", color.red());
                    telemetry.addData("Blue ", color.blue());
                    telemetry.addData("Gear ", gear.getPosition());
                    telemetry.addData("Button ", button.getValue());

                    telemetry.addData("Right ", left.getPower());
                    telemetry.addData("Left ", right.getPower());

                    telemetry.update();
                }
            }

            public void driveForward(double power) {
                left.setPower(power);
                right.setPower(power);
            }

            public void driveBack(double power) {
                left.setPower(-power);
                right.setPower(-power);
            }

            public void stopMoving() {
                left.setPower(0);
                right.setPower(0);
            }

            public void turn(double powerRight, double powerLeft) {
                left.setPower(powerLeft);
                right.setPower(powerRight);
            }

            public void turnLeft(double power) {
                left.setPower(power);
                right.setPower(-power);
            }

            public void turnRight(double power) {
                left.setPower(-power);
                right.setPower(power);
            }

            public void gunShoots(double gunPower) {
            gun.setPower(-gunPower);
            }
        }
