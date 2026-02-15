package cmd

import (
	"fmt"

	"github.com/jxwr/wheelsbot/bot-debug/pkg/serial"
	"github.com/spf13/cobra"
)

var setCmd = &cobra.Command{
	Use:   "set [param] [value]",
	Short: "Set parameter value",
	Args:  cobra.ExactArgs(2),
	RunE:  runSet,
}

func runSet(cmd *cobra.Command, args []string) error {
	param := args[0]
	value := args[1]

	port, err := findPort(portFlag)
	if err != nil {
		return err
	}

	client, err := serial.NewClient(port, baudFlag)
	if err != nil {
		return err
	}
	defer client.Close()

	// Send SET command
	client.FlushInput()
	if err := client.SendCommand(fmt.Sprintf("SET %s %s", param, value)); err != nil {
		return err
	}

	// Read response
	line, err := client.ReadLine()
	if err != nil {
		return err
	}

	if len(line) == 0 {
		return fmt.Errorf("no response")
	}

	if line[0:5] == "ERROR" {
		return fmt.Errorf("error: %s", line)
	}

	// Output CSV
	fmt.Println("success,param,value")
	if line == "OK" {
		fmt.Printf("1,%s,%s\n", param, value)
	} else {
		fmt.Printf("0,%s,%s\n", param, value)
	}

	return nil
}
