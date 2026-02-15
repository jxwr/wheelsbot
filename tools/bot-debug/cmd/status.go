package cmd

import (
	"fmt"

	"github.com/jxwr/wheelsbot/bot-debug/pkg/serial"
	"github.com/spf13/cobra"
)

var statusCmd = &cobra.Command{
	Use:   "status",
	Short: "Get system status",
	RunE:  runStatus,
}

func runStatus(cmd *cobra.Command, args []string) error {
	port, err := findPort(portFlag)
	if err != nil {
		return err
	}

	client, err := serial.NewClient(port, baudFlag)
	if err != nil {
		return err
	}
	defer client.Close()

	// Send STATUS command
	client.FlushInput()
	if err := client.SendCommand("STATUS"); err != nil {
		return err
	}

	// Read response (header + data)
	lines, err := client.ReadLines(2, 1000*1000*1000) // 1 second
	if err != nil {
		return err
	}

	if len(lines) < 2 {
		return fmt.Errorf("incomplete status response")
	}

	// Print CSV output
	fmt.Println("# System Status")
	fmt.Println(lines[0])
	fmt.Println(lines[1])

	// Get sensors
	client.FlushInput()
	if err := client.SendCommand("SENSORS"); err != nil {
		return err
	}

	lines, err = client.ReadLines(2, 1000*1000*1000)
	if err != nil {
		return err
	}

	if len(lines) >= 2 {
		fmt.Println("# Sensors")
		fmt.Println(lines[0])
		fmt.Println(lines[1])
	}

	// Get params
	client.FlushInput()
	if err := client.SendCommand("PARAMS"); err != nil {
		return err
	}

	lines, err = client.ReadLines(2, 1000*1000*1000)
	if err != nil {
		return err
	}

	if len(lines) >= 2 {
		fmt.Println("# Params")
		fmt.Println(lines[0])
		fmt.Println(lines[1])
	}

	return nil
}
