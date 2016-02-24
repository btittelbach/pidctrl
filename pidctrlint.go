// Packege pidctrl implements a PID controller.
//
// see http://en.wikipedia.org/wiki/PID_controller
package pidctrl

import (
	"fmt"
	"time"
)

const INTPID_SCALE int64 = 1 << 16

type IntMinMaxError struct {
	min, max int64
}

func (e IntMinMaxError) Error() string {
	return fmt.Sprintf("min: %v is greater than max: %v", e.min, e.max)
}

// NewIntegerPIDController returns a new IntegerPIDController using the given gain values.
func NewIntegerPIDController(p, i, d float64) *IntegerPIDController {
	return (&IntegerPIDController{outMin: -1 << 63, outMax: 1<<63 - 1}).SetPID(p, i, d)
}

// IntegerPIDController implements a PID controller.
type IntegerPIDController struct {
	p          int64     // proportional gain
	i          int64     // integral gain
	d          int64     // derrivate gain
	setpoint   int64     // current setpoint
	prevValue  int64     // last process value
	integral   int64     // integral sum
	lastUpdate time.Time // time of last update
	outMin     int64     // Output Min
	outMax     int64     // Output Max
}

// Set changes the setpoint of the controller.
func (c *IntegerPIDController) Set(setpoint int64) *IntegerPIDController {
	c.setpoint = setpoint
	return c
}

// Get returns the setpoint of the controller.
func (c *IntegerPIDController) Get() int64 {
	return c.setpoint
}

// SetPID changes the P, I, and D constants
func (c *IntegerPIDController) SetPID(p, i, d float64) *IntegerPIDController {
	c.p = int64(p * float64(INTPID_SCALE))
	c.i = int64(i * float64(INTPID_SCALE))
	c.d = int64(d * float64(INTPID_SCALE))
	return c
}

// PID returns the P, I, and D constants
func (c *IntegerPIDController) PID() (p, i, d float64) {
	return float64(c.p) / float64(INTPID_SCALE), float64(c.i) / float64(INTPID_SCALE), float64(c.d) / float64(INTPID_SCALE)
}

// SetOutputLimits sets the min and max output values
func (c *IntegerPIDController) SetOutputLimits(min, max int64) *IntegerPIDController {
	if min > max {
		panic(IntMinMaxError{min, max})
	}
	c.outMin = min * INTPID_SCALE
	c.outMax = max * INTPID_SCALE

	if c.integral > c.outMax {
		c.integral = c.outMax
	} else if c.integral < c.outMin {
		c.integral = c.outMin
	}
	return c
}

// OutputLimits returns the min and max output values
func (c *IntegerPIDController) OutputLimits() (min, max int64) {
	return c.outMin / INTPID_SCALE, c.outMax / INTPID_SCALE
}

// Update is identical to UpdateDuration, but automatically keeps track of the
// durations between updates.
func (c *IntegerPIDController) Update(value int64) int64 {
	var duration time.Duration
	if !c.lastUpdate.IsZero() {
		duration = time.Since(c.lastUpdate)
	}
	c.lastUpdate = time.Now()
	return c.UpdateDuration(value, duration)
}

// UpdateDuration updates the controller with the given value and duration since
// the last update. It returns the new output.
//
// see http://en.wikipedia.org/wiki/PID_controller#Pseudocode
func (c *IntegerPIDController) UpdateDuration(value int64, duration time.Duration) int64 {
	var (
		dt  = int64(duration.Seconds() * float64(INTPID_SCALE))
		err = c.setpoint - value
		d   int64
	)
	c.integral += (err * dt) / INTPID_SCALE * c.i
	if c.integral > c.outMax {
		c.integral = c.outMax
	} else if c.integral < c.outMin {
		c.integral = c.outMin
	}
	if dt > 0 {
		d = -((value - c.prevValue) * INTPID_SCALE / dt)
	}
	c.prevValue = value
	output := c.p*err + c.integral + (c.d * d)

	if output > c.outMax {
		output = c.outMax
	} else if output < c.outMin {
		output = c.outMin
	}

	return output / INTPID_SCALE
}

// UpdateDuration updates the controller with the given value and duration since
// the last update. It returns the new output.
//
// see http://en.wikipedia.org/wiki/PID_controller#Pseudocode
func (c *IntegerPIDController) UpdateConstInterval(value int64) int64 {
	var (
		err = c.setpoint - value
		d   = -(value - c.prevValue)
	)
	c.integral += err * c.i
	if c.integral > c.outMax {
		c.integral = c.outMax
	} else if c.integral < c.outMin {
		c.integral = c.outMin
	}
	c.prevValue = value
	output := c.p*err + c.integral + (c.d * d)

	if output > c.outMax {
		output = c.outMax
	} else if output < c.outMin {
		output = c.outMin
	}

	return output / INTPID_SCALE
}
