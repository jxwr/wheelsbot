package cmd

import (
	"encoding/csv"
	"fmt"
	"math"
	"strings"
	"time"

	"github.com/jxwr/wheelsbot/bot-debug/pkg/parser"
	"github.com/jxwr/wheelsbot/bot-debug/pkg/serial"
	"github.com/spf13/cobra"
)

func init() {
	analyzeCmd.Flags().IntVarP(&samplesFlag, "samples", "n", 200, "Number of samples")
	analyzeCmd.Flags().Float64VarP(&durationFlag, "duration", "d", 4.0, "Duration in seconds")
}

var analyzeCmd = &cobra.Command{
	Use:   "analyze",
	Short: "Analyze system stability and performance",
	RunE:  runAnalyze,
}

func runAnalyze(cmd *cobra.Command, args []string) error {
	port, err := findPort(portFlag)
	if err != nil {
		return err
	}

	client, err := serial.NewClient(port, baudFlag)
	if err != nil {
		return err
	}
	defer client.Close()

	// Calculate rate and samples
	rate := int(float64(samplesFlag) / durationFlag)
	if rate > 200 {
		rate = 200
	}
	if rate < 10 {
		rate = 10
	}

	// Start streaming
	client.FlushInput()
	if err := client.SendCommand(fmt.Sprintf("STREAM %d", rate)); err != nil {
		return err
	}

	// Collect samples
	samples := make([]*parser.SensorData, 0, samplesFlag)
	timeout := time.Duration(durationFlag+2) * time.Second

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

	// Analyze
	result := analyzeSamples(samples, durationFlag)

	// Output CSV format
	fmt.Println("# Summary")
	fmt.Println("duration_s,sample_count,sample_rate_hz")
	fmt.Printf("%.1f,%d,%.1f\n", result.Duration, result.SampleCount, result.SampleRate)

	fmt.Println("# Stability")
	fmt.Println("pitch_std,oscillation_detected,oscillation_freq_hz")
	osc := 0
	if result.OscillationDetected {
		osc = 1
	}
	fmt.Printf("%.4f,%d,%.1f\n", result.PitchStd, osc, result.OscillationFreq)

	fmt.Println("# Performance")
	fmt.Println("max_pitch_deviation,settling_time_s,overshoot_percent")
	fmt.Printf("%.4f,%.2f,%.1f\n", result.MaxPitchDev, result.SettlingTime, result.Overshoot)

	fmt.Println("# Diagnosis")
	fmt.Println("status,issue_count,recommendation")
	fmt.Printf("%s,%d,%s\n", result.Status, len(result.Issues), result.Recommendation)

	return nil
}

type AnalysisResult struct {
	Duration     float64
	SampleCount  int
	SampleRate   float64

	PitchStd           float64
	OscillationDetected bool
	OscillationFreq    float64

	MaxPitchDev  float64
	SettlingTime float64
	Overshoot    float64

	Status         string
	Issues         []string
	Recommendation string
}

func analyzeSamples(samples []*parser.SensorData, duration float64) AnalysisResult {
	n := len(samples)
	result := AnalysisResult{
		Duration:    duration,
		SampleCount: n,
		SampleRate:  float64(n) / duration,
	}

	if n == 0 {
		result.Status = "NO_DATA"
		return result
	}

	// Calculate pitch statistics
	var sum, sumSq, maxAbs float64
	pitches := make([]float64, n)
	for i, s := range samples {
		p := s.Pitch
		pitches[i] = p
		sum += p
		sumSq += p * p
		if math.Abs(p) > maxAbs {
			maxAbs = math.Abs(p)
		}
	}

	mean := sum / float64(n)
	variance := (sumSq / float64(n)) - (mean * mean)
	std := math.Sqrt(variance)

	result.PitchStd = std
	result.MaxPitchDev = maxAbs

	// Detect oscillation (simple zero-crossing method)
	zeroCrossings := 0
	for i := 1; i < n; i++ {
		if (pitches[i-1] > mean && pitches[i] < mean) ||
			(pitches[i-1] < mean && pitches[i] > mean) {
			zeroCrossings++
		}
	}

	if zeroCrossings > n/10 {
		result.OscillationDetected = true
		result.OscillationFreq = float64(zeroCrossings) / (2 * duration)
	}

	// Estimate settling time (time to reach within 2*std of mean)
	threshold := 2 * std
	settled := false
	for i := n / 2; i < n; i++ {
		if math.Abs(pitches[i]-mean) > threshold {
			settled = false
			break
		}
		settled = true
	}

	if settled {
		result.SettlingTime = duration / 2
	} else {
		result.SettlingTime = duration
	}

	// Overshoot (max deviation in first half)
	maxFirst := 0.0
	for i := 0; i < n/2; i++ {
		if math.Abs(pitches[i]) > maxFirst {
			maxFirst = math.Abs(pitches[i])
		}
	}
	result.Overshoot = (maxFirst / (std + 0.001)) * 100

	// Diagnosis
	result.Issues = []string{}

	if result.OscillationDetected {
		result.Status = "OSCILLATING"
		result.Issues = append(result.Issues, "oscillation_detected")
		result.Recommendation = "Reduce angle_kp or increase angle_gyro_kd"
	} else if std > 0.05 {
		result.Status = "UNSTABLE"
		result.Issues = append(result.Issues, "high_variance")
		result.Recommendation = "Increase damping (angle_gyro_kd)"
	} else if std < 0.01 {
		result.Status = "STABLE"
		result.Recommendation = "Consider increasing velocity_kp for faster response"
	} else {
		result.Status = "ACCEPTABLE"
		result.Recommendation = "System is functional but could be tuned"
	}

	return result
}

func parseCSVLine(line string) ([]string, error) {
	r := csv.NewReader(strings.NewReader(line))
	return r.Read()
}
