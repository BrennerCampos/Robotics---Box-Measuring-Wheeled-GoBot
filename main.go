package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
)

func main() {
	//We create the adaptors to connect the GoPiGo3 board with the Raspberry Pi 3
	raspiAdapter := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdapter)

	// create sensor drivers here
	lightSensorFront := aio.NewGroveLightSensorDriver(gopigo3, "AD_1_1")
	secondLightSensor := aio.NewGroveLightSensorDriver(gopigo3, "AD_2_1")
	lidarSensorFront := i2c.NewLIDARLiteDriver(raspiAdapter)

	// local variable that holds anonymous function
	// robot framework that creates a new thread and run this fucntion
	mainRobotFunc := func() {
		robotRunLoop(lightSensorFront, secondLightSensor, lidarSensorFront, gopigo3)
	}

	// crux of gobot framework, factory function to create a new robot
	robot := gobot.NewRobot("gopigo3sensorChecker", // robot name
		[]gobot.Connection{raspiAdapter},                             // slice of connected robots
		[]gobot.Device{gopigo3, lightSensorFront, secondLightSensor}, // slice of sensors or actuators for robot
		mainRobotFunc) // variable that holds the robots function

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
	gpg.SetMotorDps(g.MOTOR_LEFT, 40)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 40)
}

func stopMove(gpg *g.Driver) {
	gpg.SetMotorDps(g.MOTOR_LEFT, 0)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 0)
}

func robotRunLoop(frontLight *aio.GroveLightSensorDriver, secondLight *aio.GroveLightSensorDriver, frontLidar *i2c.LIDARLiteDriver, gpg *g.Driver) {

	gpg.SetLED(3, 0, 0, 255) // light on - blue (led 4 might be under chip, don't know where led 1 and 2 is or if it exists)
	count := 0

	// for(ever) loop to keep robot running
	for {
		// values of sensors
		frontLightVal, err := frontLight.Read()
		if err != nil {
			fmt.Errorf("Error reading front light sensor %+v", err)
		}
		secondLightVal, err := secondLight.Read()
		if err != nil {
			fmt.Errorf("Error reading second light sensor %+v", err)
		}

		err = frontLidar.Start()
		frontLidarVal, err := frontLidar.Distance()

		count++

		// print values into console
		fmt.Println("______________________________") // 30 characters
		fmt.Printf("|___________%-5d____________|\n", count)
		fmt.Printf("|%-20s:   %-4d|\n", "front light sensor", frontLightVal)
		fmt.Printf("|%-20s:   %-4d|\n", "second light sensor", secondLightVal)
		fmt.Printf("|%-20s:   %-4d|\n", "front lidar sensor", frontLidarVal)

		time.Sleep(time.Second)

		if frontLightVal < 500 && secondLightVal < 500 {
			turnLeft(gpg)
			gpg.SetLED(3, 0, 0, 0) // off

			// find light source - this will assume no objects are in the way
		} else if frontLightVal < 2900 && frontLightVal > 900 {
			moveForward(gpg)
			gpg.SetLED(3, 0, 255, 0) // green

		} else if frontLightVal >= 2900 {
			stopMove(gpg)
		}

		// if we are close to the object, we will stop for now
		if frontLidarVal <= 65 { //the higher the number, the further it stops
			stopMove(gpg)
			gpg.SetLED(3, 255, 128, 0) // orange
		}

	}
}
