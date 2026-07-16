// Package candb defines the ZC2X CAN2 signal database: the message/signal
// layout MoTeC M1 Tune (or the ecu/ firmware simulator) transmits on CAN2.
// This table mirrors docs/architecture/zc2x-can2.dbc byte-for-byte — keep
// both in sync by hand when either changes.
package candb

import (
	"encoding/binary"
	"errors"
	"fmt"
)

// ValueType is the wire encoding of a signal's raw bytes.
type ValueType int

const (
	Uint8 ValueType = iota
	Int16
	Uint16
	Uint32
	Int32
)

// Signal describes one named value packed into a CAN message payload.
// Physical value = raw*Scale + Bias (Bias is 0 for every current signal;
// kept for parity with the DBC's (factor,offset) pair).
type Signal struct {
	Name       string
	ByteOffset int
	Type       ValueType
	Scale      float64
	Bias       float64
	Unit       string
}

// Message describes one CAN message: its ID, DLC, and ordered signals.
type Message struct {
	ID      uint32
	Name    string
	DLC     uint8
	Signals []Signal
}

// ErrUnknownMessage is returned by DecodeSignals when canID has no entry in
// this database. This is expected, not exceptional: OBU/RSU forward every
// CAN ID unfiltered (platform/esp-idf/firmware/obu/main/main.c can_init()),
// including IDs with no signal definition (yet).
var ErrUnknownMessage = errors.New("candb: unknown CAN message id")

// Names follow MoTeC's generic dash/logger CAN convention: CD# = Chassis
// Dynamics, PE# = Powertrain/Engine, numbered per message rather than named
// after their exact field mix.
var messages = []Message{
	{ID: 0x100, Name: "CD1", DLC: 8, Signals: []Signal{
		{Name: "WheelSpeedFL", ByteOffset: 0, Type: Uint16, Scale: 0.1, Unit: "km/h"},
		{Name: "WheelSpeedFR", ByteOffset: 2, Type: Uint16, Scale: 0.1, Unit: "km/h"},
		{Name: "WheelSpeedRL", ByteOffset: 4, Type: Uint16, Scale: 0.1, Unit: "km/h"},
		{Name: "WheelSpeedRR", ByteOffset: 6, Type: Uint16, Scale: 0.1, Unit: "km/h"},
	}},
	{ID: 0x110, Name: "CD2", DLC: 8, Signals: []Signal{
		{Name: "SteeringAngle", ByteOffset: 0, Type: Int16, Scale: 0.1, Unit: "deg"},
		{Name: "GForceLateral", ByteOffset: 2, Type: Int16, Scale: 0.001, Unit: "g"},
		{Name: "GForceLongitudinal", ByteOffset: 4, Type: Int16, Scale: 0.001, Unit: "g"},
		{Name: "GroundSpeed", ByteOffset: 6, Type: Uint16, Scale: 0.1, Unit: "km/h"},
	}},
	{ID: 0x120, Name: "CD3", DLC: 4, Signals: []Signal{
		{Name: "BrakePressureFront", ByteOffset: 0, Type: Uint16, Scale: 0.1, Unit: "bar"},
		{Name: "BrakePressureRear", ByteOffset: 2, Type: Uint16, Scale: 0.1, Unit: "bar"},
	}},
	{ID: 0x200, Name: "PE1", DLC: 8, Signals: []Signal{
		{Name: "EngineRPM", ByteOffset: 0, Type: Uint16, Scale: 1, Unit: "rpm"},
		{Name: "ThrottlePosition", ByteOffset: 2, Type: Uint8, Scale: 1, Unit: "%"},
		{Name: "Lambda1", ByteOffset: 3, Type: Uint16, Scale: 0.001, Unit: ""},
		{Name: "ManifoldAirPressure", ByteOffset: 5, Type: Uint16, Scale: 0.1, Unit: "kPa"},
		{Name: "Gear", ByteOffset: 7, Type: Uint8, Scale: 1, Unit: ""},
	}},
	{ID: 0x210, Name: "PE2", DLC: 8, Signals: []Signal{
		{Name: "EngineCoolantTemperature", ByteOffset: 0, Type: Int16, Scale: 0.1, Unit: "degC"},
		{Name: "EngineOilTemperature", ByteOffset: 2, Type: Int16, Scale: 0.1, Unit: "degC"},
		{Name: "ManifoldAirTemperature", ByteOffset: 4, Type: Int16, Scale: 0.1, Unit: "degC"},
		{Name: "EngineOilPressure", ByteOffset: 6, Type: Uint16, Scale: 0.1, Unit: "bar"},
	}},
	{ID: 0x211, Name: "PE3", DLC: 6, Signals: []Signal{
		{Name: "FuelLinePressure", ByteOffset: 0, Type: Uint16, Scale: 0.1, Unit: "bar"},
		{Name: "FuelUsedRaw", ByteOffset: 2, Type: Uint32, Scale: 1, Unit: ""},
	}},
	{ID: 0x220, Name: "PE4", DLC: 6, Signals: []Signal{
		{Name: "ExhaustCylinderTemperature1", ByteOffset: 0, Type: Int16, Scale: 1, Unit: "degC"},
		{Name: "ExhaustCylinderTemperature2", ByteOffset: 2, Type: Int16, Scale: 1, Unit: "degC"},
		{Name: "ExhaustCylinderTemperature3", ByteOffset: 4, Type: Int16, Scale: 1, Unit: "degC"},
	}},
	{ID: 0x300, Name: "GPS1", DLC: 8, Signals: []Signal{
		{Name: "Latitude", ByteOffset: 0, Type: Int32, Scale: 0.0000001, Unit: "deg"},
		{Name: "Longitude", ByteOffset: 4, Type: Int32, Scale: 0.0000001, Unit: "deg"},
	}},
	{ID: 0x310, Name: "GPS2", DLC: 4, Signals: []Signal{
		{Name: "Altitude", ByteOffset: 0, Type: Int16, Scale: 0.1, Unit: "m"},
		{Name: "GPSSpeed", ByteOffset: 2, Type: Uint16, Scale: 0.1, Unit: "km/h"},
	}},
}

var messagesByID = func() map[uint32]Message {
	m := make(map[uint32]Message, len(messages))
	for _, msg := range messages {
		m[msg.ID] = msg
	}
	return m
}()

// MessageByID looks up a message definition by CAN ID.
func MessageByID(canID uint32) (Message, bool) {
	msg, ok := messagesByID[canID]
	return msg, ok
}

// DecodeSignals decodes data (a CAN frame payload, up to 8 bytes) into named
// engineering-unit values, per the message definition for canID. Returns
// ErrUnknownMessage if canID isn't in this database (expected for CAN IDs
// with no ZC2X signal definition — callers should treat that as "skip",
// not a real error).
func DecodeSignals(canID uint32, data []byte) (map[string]float64, error) {
	msg, ok := messagesByID[canID]
	if !ok {
		return nil, fmt.Errorf("%w: 0x%03x", ErrUnknownMessage, canID)
	}
	if len(data) < int(msg.DLC) {
		return nil, fmt.Errorf("candb: message %s (0x%03x) needs %d bytes, got %d",
			msg.Name, canID, msg.DLC, len(data))
	}

	out := make(map[string]float64, len(msg.Signals))
	for _, sig := range msg.Signals {
		raw, err := sig.decodeRaw(data)
		if err != nil {
			return nil, fmt.Errorf("candb: signal %s: %w", sig.Name, err)
		}
		out[sig.Name] = raw*sig.Scale + sig.Bias
	}
	return out, nil
}

func (s Signal) decodeRaw(data []byte) (float64, error) {
	end := s.ByteOffset + s.byteLen()
	if end > len(data) {
		return 0, fmt.Errorf("byte offset %d+%d exceeds payload length %d", s.ByteOffset, s.byteLen(), len(data))
	}
	switch s.Type {
	case Uint8:
		return float64(data[s.ByteOffset]), nil
	case Int16:
		return float64(int16(binary.LittleEndian.Uint16(data[s.ByteOffset:]))), nil
	case Uint16:
		return float64(binary.LittleEndian.Uint16(data[s.ByteOffset:])), nil
	case Uint32:
		return float64(binary.LittleEndian.Uint32(data[s.ByteOffset:])), nil
	case Int32:
		return float64(int32(binary.LittleEndian.Uint32(data[s.ByteOffset:]))), nil
	default:
		return 0, fmt.Errorf("unknown value type %d", s.Type)
	}
}

func (s Signal) byteLen() int {
	switch s.Type {
	case Uint8:
		return 1
	case Int16, Uint16:
		return 2
	case Uint32, Int32:
		return 4
	default:
		return 0
	}
}
