package serial

import (
	"bufio"
	"fmt"
	"strings"
	"time"

	"go.bug.st/serial"
)

type Client struct {
	port   serial.Port
	reader *bufio.Reader
}

func NewClient(portName string, baud int) (*Client, error) {
	mode := &serial.Mode{
		BaudRate: baud,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open port: %w", err)
	}

	// Give device time to settle
	time.Sleep(100 * time.Millisecond)

	return &Client{
		port:   port,
		reader: bufio.NewReader(port),
	}, nil
}

func (c *Client) Close() error {
	return c.port.Close()
}

func (c *Client) SendCommand(cmd string) error {
	_, err := c.port.Write([]byte(cmd + "\n"))
	if err != nil {
		return fmt.Errorf("failed to send command: %w", err)
	}
	return nil
}

func (c *Client) ReadLine() (string, error) {
	line, err := c.reader.ReadString('\n')
	if err != nil {
		return "", err
	}
	return strings.TrimSpace(line), nil
}

func (c *Client) ReadLines(n int, timeout time.Duration) ([]string, error) {
	lines := make([]string, 0, n)
	deadline := time.Now().Add(timeout)

	for len(lines) < n {
		if time.Now().After(deadline) {
			break
		}

		c.port.SetReadTimeout(100 * time.Millisecond)
		line, err := c.ReadLine()
		if err != nil {
			continue
		}

		if line != "" {
			lines = append(lines, line)
		}
	}

	return lines, nil
}

func (c *Client) FlushInput() {
	// Read and discard all pending input
	c.port.SetReadTimeout(50 * time.Millisecond)
	buf := make([]byte, 1024)
	for {
		_, err := c.port.Read(buf)
		if err != nil {
			break
		}
	}
	c.reader = bufio.NewReader(c.port)
}
