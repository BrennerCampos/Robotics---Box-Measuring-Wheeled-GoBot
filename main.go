package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
)

func main() {
	//We create the adaptors to connect the GoPiGo3 board with the Raspberry Pi 3
	raspiAdapter := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdapter)

	lidarSensor := i2c.NewLIDARLiteDriver(raspiAdapter)

	// local variable that holds anonymous function
	// robot framework that creates a new thread and run this function
	mainRobotFunc := func() {
		robotRunLoop(lidarSensor, gopigo3)
	}

	// crux of gobot framework, factory function to create a new robot
	robot := gobot.NewRobot("gopigo3sensorChecker", // robot name
		[]gobot.Connection{raspiAdapter},     // slice of connected robots
		[]gobot.Device{gopigo3, lidarSensor}, // slice of sensors or actuators for robot
		mainRobotFunc)                        // variable that holds the robots function

	robot.Start() //run robot function
}

func turnLeft(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 35)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 70)
}

func turnRight(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 70)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 35)
}

func moveForward(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 70)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 70)
}

func stopMove(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 0)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 0)
}

func robotRunLoop(lidarSensor *i2c.LIDARLiteDriver, gpg *g.Driver) {

	gpg.SetLED(3, 0, 0, 255) // light on - blue (led 4 might be under chip, don't know where led 1 and 2 is or if it exists)
	count := 0

	// --   START

	//  - first, check lidar to see if it's facing the box or not

	// -- LOOP 1 - CHECKING
	//  - if not, move forward until we see 'something' (the box), and start recording distance

	// -- LOOP 2 - MOVING and STOPPING
	//  - if yes, move forward until we no longer see the box, move forward to compensate for body of robot + 25 cm distance worth
	// - then turn 90 degrees clockwise
	// after turn, continue moving forward, recording distance as soon as it sees the box (around 30-50ish from our calculations)
	// check that the lidar values stay relatively the same, otherwise if they are rising or diminishing it might mean the robot is not perpendicular

	// when done with second side, stop robot, do calculations to get area + perimeter

	// --    END

	/*
		look into:
		setMotorPosition - set left or right wheel a # of degrees
		getMotorEncoder - read motor encoder in degrees
	*/

	for { // for(ever) loop to keep robot running

		battery, err := gpg.GetBatteryVoltage()
		if err != nil {
			fmt.Errorf("Unable to get battery voltage %+v", err)
		}

		// values of sensors
		err = lidarSensor.Start()
		lidarVal, err := lidarSensor.Distance()
		if err != nil {
			fmt.Errorf("lidar sensor reading error %+v", err)
		}
		count++

		leftMotor, err := gpg.GetMotorEncoder(g.MOTOR_LEFT)
		if err != nil {
			fmt.Errorf("left motor encorder not reading %+v", err)
		}

		// print values into console
		fmt.Println("______________________________") // 30 characters
		fmt.Printf("|___________%-5d____________|\n", count)
		fmt.Printf("|%-20s:   %-4f|\n", "Battery (v)", battery)
		fmt.Printf("|%-20s:   %-4d|\n", "lidar sensor", lidarVal)
		fmt.Printf("|%-20s:   %-4d|\n", "left wheel (degrees)", leftMotor%360)

		time.Sleep(time.Second)

		if lidarVal >= 80 {
			turnLeft(gpg)
		} else if lidarVal < 30 {
			turnRight(gpg)

		} else if lidarVal >= 25 && lidarVal < 80 {
			moveForward(gpg)
		}

		if lidarVal > 10 && lidarVal <= 60 {
			gpg.SetLED(3, 0, 255, 0) // green
		} else {
			gpg.SetLED(3, 255, 0, 0) // red
		}

		// color values based on lidar values
		if lidarVal <= 10 { //the higher the number, the further it stops
			gpg.SetLED(3, 255, 128, 0) // orange
		} else if lidarVal > 10 && lidarVal <= 60 {
			gpg.SetLED(3, 0, 255, 0) // green
		} else {
			gpg.SetLED(3, 255, 0, 0) // red
		}

	}
}
