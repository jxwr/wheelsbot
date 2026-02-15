package parser

import (
	"encoding/csv"
	"fmt"
	"io"
	"strconv"
	"strings"
)

type SensorData struct {
	Timestamp    uint32
	Pitch        float64
	PitchRate    float64
	YawRate      float64
	WheelVelL    float64
	WheelVelR    float64
	MotorL       float64
	MotorR       float64
	Voltage      float64
	State        int
	Faults       uint32
}

func ParseCSVLine(line string) ([]string, error) {
	r := csv.NewReader(strings.NewReader(line))
	record, err := r.Read()
	if err != nil && err != io.EOF {
		return nil, err
	}
	return record, nil
}

func ParseDataLine(line string) (*SensorData, error) {
	fields, err := ParseCSVLine(line)
	if err != nil {
		return nil, err
	}

	if len(fields) < 11 || fields[0] != "DATA" {
		return nil, fmt.Errorf("invalid DATA line format")
	}

	data := &SensorData{}
	var u uint64
	var f float64

	// timestamp
	if u, err = strconv.ParseUint(fields[1], 10, 32); err != nil {
		return nil, err
	}
	data.Timestamp = uint32(u)

	// pitch
	if f, err = strconv.ParseFloat(fields[2], 64); err != nil {
		return nil, err
	}
	data.Pitch = f

	// pitch_rate
	if f, err = strconv.ParseFloat(fields[3], 64); err != nil {
		return nil, err
	}
	data.PitchRate = f

	// yaw_rate
	if f, err = strconv.ParseFloat(fields[4], 64); err != nil {
		return nil, err
	}
	data.YawRate = f

	// wheel_vel_l
	if f, err = strconv.ParseFloat(fields[5], 64); err != nil {
		return nil, err
	}
	data.WheelVelL = f

	// wheel_vel_r
	if f, err = strconv.ParseFloat(fields[6], 64); err != nil {
		return nil, err
	}
	data.WheelVelR = f

	// motor_l
	if f, err = strconv.ParseFloat(fields[7], 64); err != nil {
		return nil, err
	}
	data.MotorL = f

	// motor_r
	if f, err = strconv.ParseFloat(fields[8], 64); err != nil {
		return nil, err
	}
	data.MotorR = f

	// voltage
	if f, err = strconv.ParseFloat(fields[9], 64); err != nil {
		return nil, err
	}
	data.Voltage = f

	// state
	var s int64
	if s, err = strconv.ParseInt(fields[10], 10, 32); err != nil {
		return nil, err
	}
	data.State = int(s)

	// faults
	if u, err = strconv.ParseUint(fields[11], 10, 32); err != nil {
		return nil, err
	}
	data.Faults = uint32(u)

	return data, nil
}

func FormatCSV(fields []string) string {
	var buf strings.Builder
	for i, f := range fields {
		if i > 0 {
			buf.WriteByte(',')
		}
		buf.WriteString(f)
	}
	return buf.String()
}
