package cmd

import (
	"github.com/spf13/cobra"
)

var (
	portFlag string
	baudFlag int
)

var rootCmd = &cobra.Command{
	Use:   "bot-debug",
	Short: "Balance bot debug CLI for AI agent",
	Long:  `CSV-based debug tool for balance robot parameter tuning and monitoring.`,
}

func init() {
	// Global flags
	rootCmd.PersistentFlags().StringVarP(&portFlag, "port", "p", "/dev/cu.usbmodem*", "Serial port")
	rootCmd.PersistentFlags().IntVarP(&baudFlag, "baud", "b", 115200, "Baud rate")

	// Add subcommands
	rootCmd.AddCommand(readCmd)
	rootCmd.AddCommand(statusCmd)
	rootCmd.AddCommand(getCmd)
	rootCmd.AddCommand(setCmd)
	rootCmd.AddCommand(streamCmd)
	rootCmd.AddCommand(analyzeCmd)
}

func Execute() error {
	return rootCmd.Execute()
}
