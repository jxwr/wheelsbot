package cmd

import (
	"fmt"

	"github.com/jxwr/wheelsbot/bot-debug/pkg/serial"
	"github.com/spf13/cobra"
)

var getCmd = &cobra.Command{
	Use:   "get [param]",
	Short: "Get parameter value",
	Args:  cobra.ExactArgs(1),
	RunE:  runGet,
}

func runGet(cmd *cobra.Command, args []string) error {
	param := args[0]

	port, err := findPort(portFlag)
	if err != nil {
		return err
	}

	client, err := serial.NewClient(port, baudFlag)
	if err != nil {
		return err
	}
	defer client.Close()

	// Send GET command
	client.FlushInput()
	if err := client.SendCommand(fmt.Sprintf("GET %s", param)); err != nil {
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

	// Output CSV (VALUE,param,value)
	fmt.Println("param,value")
	// Parse VALUE,param,value and output just param,value
	fields, err := parseCSVLine(line)
	if err == nil && len(fields) == 3 && fields[0] == "VALUE" {
		fmt.Printf("%s,%s\n", fields[1], fields[2])
	} else {
		fmt.Println(line)
	}

	return nil
}
