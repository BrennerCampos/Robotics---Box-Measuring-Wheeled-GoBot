package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
)

var (
	fwdLoopCounter int // used to keep track how many times we circle around the box
	tally          int // used to count increments of wheel rotation for measuring distance
)

func main() {

	// https://github.com/hybridgroup/gobot/blob/release/platforms/dexter/gopigo3/driver.go#L88

	//We create the adaptors to connect the GoPiGo3 board with the Raspberry Pi 3
	raspiAdapter := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdapter)

	lidarSensor := i2c.NewLIDARLiteDriver(raspiAdapter)
	fwdLoopCounter = 0
	tally = 0

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

func takeTurn(lidarSensor *i2c.LIDARLiteDriver, gpg *g.Driver) {
	counter := 3

	fmt.Println("90 degree turn in: ")
	for counter > 0 {
		time.Sleep(time.Second)
		fmt.Println(counter)
		counter--
	}

	stopMove(gpg)

	gpg.SetMotorDps(g.MOTOR_LEFT, 0)
	gpg.SetMotorDps(g.MOTOR_RIGHT, 165)
	time.Sleep(time.Second)

	//err := lidarSensor.Start()
	//lidarVal, err := lidarSensor.Distance()
	//if err != nil {
	//	fmt.Errorf("lidar sensor reading error %+v", err)
	//}
	//fmt.Println("90 degree turn complete????") // will confirm if loop below is necessary
	//
	//for lidarVal > 50 {
	//
	//	err = lidarSensor.Start()
	//	lidarVal, err = lidarSensor.Distance()
	//	gpg.SetMotorDps(g.MOTOR_RIGHT, 160)
	//	time.Sleep(time.Second)
	//
	//	fmt.Println("____________pause___________") // 30 characters
	//	fmt.Printf("|%-20s:   %-4d|\n", "lidar sensor", lidarVal)
	//	moveForward(gpg)
	//}

	fmt.Println("90 degree turn complete!")

	counter = 0
	for counter < 1 {
		time.Sleep(time.Second)
		fmt.Println("extra delay")
		counter++
	}
	fmt.Println()
}

func forwardLoop(gpg *g.Driver) {
	counter := 2
	stopMove(gpg)
	moveForward(gpg)

	fmt.Println("forward loop end in: ")
	for counter > 0 {
		time.Sleep(time.Second)
		fmt.Println(counter)
		counter--
	}
	fmt.Println("end forward loop\n")

	tally = 0
	fwdLoopCounter++
}

func robotRunLoop(lidarSensor *i2c.LIDARLiteDriver, gpg *g.Driver) {

	gpg.SetLED(3, 0, 0, 255) // light on - blue (led 4 might be under chip, don't know where led 1 and 2 is or if it exists)
	count := 0
	dimensions := [2]int{0, 0}

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

		// start tallying degree rotations for measurement
		if fwdLoopCounter == 1 || fwdLoopCounter == 2 {
			if lidarVal >= 10 && lidarVal <= 25 { // precautionary if statement, might not need
				if leftMotor%5 == 0 {
					tally++
				}
			}
		}

		// print values into console
		fmt.Println("______________________________") // 30 characters
		fmt.Printf("|___________%-5d____________|\n", count)

		// battery warning
		if battery <= 9 {
			fmt.Printf("|%-20s:   %-4f|\n", "Battery low!!", battery)
		} else {
			fmt.Printf("|%-20s:   %-4f|\n", "Battery (v)", battery)
		}

		fmt.Printf("|%-20s:   %-4d|\n", "lidar sensor", lidarVal)
		fmt.Printf("|%-20s:   %-4d|\n", "left wheel (degrees)", leftMotor%360)
		fmt.Printf("|%-20s:   %-4d|\n", "fwd counter", fwdLoopCounter)
		fmt.Printf("|%-20s:   %-4d|\n", "one side (mm)", dimensions[0])
		fmt.Printf("|%-20s:   %-4d|\n", "other side (mm)", dimensions[1])

		time.Sleep(time.Millisecond)

		if lidarVal >= 70 {
			fmt.Println("entering turning loop")
			takeTurn(lidarSensor, gpg)

			fmt.Println("entering forward loop")
			forwardLoop(gpg)
		} else if lidarVal >= 10 && lidarVal < 70 {
			moveForward(gpg)
		}

		// color values based on lidar values
		if lidarVal <= 10 {
			gpg.SetLED(3, 255, 128, 0) // orange
		} else if lidarVal > 10 && lidarVal <= 30 {
			gpg.SetLED(3, 0, 255, 0) // green
		} else {
			gpg.SetLED(3, 255, 0, 0) // red
		}

		// 360 degrees = ~ 150 - 170mm = 72 tallys ( based on %5)
		//		2.2 mm per tally
		// the %5 might not work since the values are random and may not be divisible by 5, we could be missing values

		if fwdLoopCounter == 1 {
			dimensions[0] = int(float64(tally) * 22)
		} else if fwdLoopCounter == 2 {
			dimensions[1] = int(float64(tally) * 22)
		} else if fwdLoopCounter > 3 {
			fmt.Println("measurement complete!")
			fmt.Println("The perimeter of the box is:", dimensions[0]*dimensions[1], "mm")
		}

	}
}
