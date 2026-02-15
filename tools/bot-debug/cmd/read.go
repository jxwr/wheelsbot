package cmd

import (
	"fmt"
	"path/filepath"
	"time"

	"github.com/jxwr/wheelsbot/bot-debug/pkg/parser"
	"github.com/jxwr/wheelsbot/bot-debug/pkg/serial"
	"github.com/spf13/cobra"
)

var readCmd = &cobra.Command{
	Use:   "read",
	Short: "Read one sensor sample",
	RunE:  runRead,
}

func runRead(cmd *cobra.Command, args []string) error {
	// Find serial port
	port, err := findPort(portFlag)
	if err != nil {
		return err
	}

	client, err := serial.NewClient(port, baudFlag)
	if err != nil {
		return err
	}
	defer client.Close()

	// Start streaming
	client.FlushInput()
	if err := client.SendCommand("STREAM 50"); err != nil {
		return err
	}

	// Read one DATA line
	for i := 0; i < 20; i++ {
		line, err := client.ReadLine()
		if err != nil {
			time.Sleep(50 * time.Millisecond)
			continue
		}

		if len(line) > 0 && line[0:4] == "DATA" {
			data, err := parser.ParseDataLine(line)
			if err != nil {
				return fmt.Errorf("failed to parse data: %w", err)
			}

			// Output CSV
			fmt.Println("timestamp,pitch,pitch_rate,yaw_rate,wheel_vel_l,wheel_vel_r,motor_l,motor_r,voltage,state,faults")
			fmt.Printf("%d,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
				data.Timestamp,
				data.Pitch,
				data.PitchRate,
				data.YawRate,
				data.WheelVelL,
				data.WheelVelR,
				data.MotorL,
				data.MotorR,
				data.Voltage,
				data.State,
				data.Faults,
			)

			// Stop streaming
			client.SendCommand("STREAM 0")
			return nil
		}
	}

	return fmt.Errorf("timeout waiting for data")
}

func findPort(pattern string) (string, error) {
	matches, err := filepath.Glob(pattern)
	if err != nil {
		return "", err
	}
	if len(matches) == 0 {
		return "", fmt.Errorf("no matching port found for pattern: %s", pattern)
	}
	return matches[0], nil
}
