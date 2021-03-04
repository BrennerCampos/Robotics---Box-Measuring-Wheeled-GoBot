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
	gpg.SetMotorDps(g.MOTOR_LEFT, 65)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 0)
}

func turnRight(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 0)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 80)
}

func moveForward(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 50)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 50)
}

func stopMove(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 0)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 0)
}

func robotRunLoop(lidarSensor *i2c.LIDARLiteDriver, gpg *g.Driver) {

	gpg.SetLED(3, 0, 0, 255) // light on - blue (led 4 might be under chip, don't know where led 1 and 2 is or if it exists)
	count := 0

	for { // for(ever) loop to keep robot running

		// values of sensors
		err := lidarSensor.Start()
		lidarVal, err := lidarSensor.Distance()
		if err != nil {
			fmt.Errorf("lidar sensor reading error %+v", err)
		}
		count++

		// print values into console
		fmt.Println("______________________________") // 30 characters
		fmt.Printf("|___________%-5d____________|\n", count)
		fmt.Printf("|%-20s:   %-4d|\n", "front lidar sensor", lidarVal)

		time.Sleep(time.Second)

		//moveForward(gpg)

		// if we are close to the object, we will stop for now
		if lidarVal <= 65 { //the higher the number, the further it stops
			gpg.SetLED(3, 255, 128, 0)
		} else if lidarVal > 65 && lidarVal <= 90 {
			gpg.SetLED(3, 0, 255, 0) // orange
		} else {
			gpg.SetLED(3, 255, 0, 0) // orange
		}

	}
}
