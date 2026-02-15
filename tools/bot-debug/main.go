package main

import (
	"fmt"
	"os"

	"github.com/jxwr/wheelsbot/bot-debug/cmd"
)

func main() {
	if err := cmd.Execute(); err != nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
		os.Exit(1)
	}
}
