package cmd

import (
	"fmt"
	"math"
	"time"

	"github.com/jxwr/wheelsbot/bot-debug/pkg/parser"
	"github.com/jxwr/wheelsbot/bot-debug/pkg/serial"
	"github.com/spf13/cobra"
)

var (
	samplesFlag  int
	rateFlag     int
	durationFlag float64
)

func init() {
	streamCmd.Flags().IntVarP(&samplesFlag, "samples", "n", 100, "Number of samples")
	streamCmd.Flags().IntVarP(&rateFlag, "rate", "r", 50, "Sample rate (Hz)")
	streamCmd.Flags().Float64VarP(&durationFlag, "duration", "d", 0, "Duration in seconds (overrides samples)")
}

var streamCmd = &cobra.Command{
	Use:   "stream",
	Short: "Stream sensor data with statistics",
	RunE:  runStream,
}

func runStream(cmd *cobra.Command, args []string) error {
	port, err := findPort(portFlag)
	if err != nil {
		return err
	}

	client, err := serial.NewClient(port, baudFlag)
	if err != nil {
		return err
	}
	defer client.Close()

	// Calculate samples if duration is specified
	if durationFlag > 0 {
		samplesFlag = int(durationFlag * float64(rateFlag))
	}

	// Start streaming
	client.FlushInput()
	if err := client.SendCommand(fmt.Sprintf("STREAM %d", rateFlag)); err != nil {
		return err
	}

	// Collect samples
	samples := make([]*parser.SensorData, 0, samplesFlag)
	timeout := time.Duration(samplesFlag/rateFlag+5) * time.Second

	deadline := time.Now().Add(timeout)
	for len(samples) < samplesFlag && time.Now().Before(deadline) {
		line, err := client.ReadLine()
		if err != nil {
			time.Sleep(10 * time.Millisecond)
			continue
		}

		if len(line) > 4 && line[0:4] == "DATA" {
			data, err := parser.ParseDataLine(line)
			if err != nil {
				continue
			}
			samples = append(samples, data)
		}
	}

	// Stop streaming
	client.SendCommand("STREAM 0")

	if len(samples) == 0 {
		return fmt.Errorf("no samples collected")
	}

	// Output samples
	fmt.Println("# Samples")
	fmt.Println("timestamp,pitch,pitch_rate,yaw_rate,wheel_vel_l,wheel_vel_r,motor_l,motor_r,voltage,state,faults")
	for _, s := range samples {
		fmt.Printf("%d,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
			s.Timestamp, s.Pitch, s.PitchRate, s.YawRate,
			s.WheelVelL, s.WheelVelR, s.MotorL, s.MotorR,
			s.Voltage, s.State, s.Faults)
	}

	// Calculate statistics
	stats := calculateStats(samples)

	// Output stats
	fmt.Println("# Stats")
	fmt.Println("stat,pitch_mean,pitch_std,pitch_max,pitch_min")
	fmt.Printf("value,%.4f,%.4f,%.4f,%.4f\n",
		stats.PitchMean, stats.PitchStd, stats.PitchMax, stats.PitchMin)

	return nil
}

type Stats struct {
	PitchMean float64
	PitchStd  float64
	PitchMax  float64
	PitchMin  float64
}

func calculateStats(samples []*parser.SensorData) Stats {
	if len(samples) == 0 {
		return Stats{}
	}

	var sum, sumSq, max, min float64
	max = -math.MaxFloat64
	min = math.MaxFloat64

	for _, s := range samples {
		p := s.Pitch
		sum += p
		sumSq += p * p
		if p > max {
			max = p
		}
		if p < min {
			min = p
		}
	}

	n := float64(len(samples))
	mean := sum / n
	variance := (sumSq / n) - (mean * mean)
	std := math.Sqrt(variance)

	return Stats{
		PitchMean: mean,
		PitchStd:  std,
		PitchMax:  max,
		PitchMin:  min,
	}
}
