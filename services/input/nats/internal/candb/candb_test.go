package candb

import (
	"math"
	"testing"
)

const epsilon = 1e-9

func approxEqual(a, b float64) bool {
	return math.Abs(a-b) < epsilon
}

func TestDecodeSignals_EngineCore(t *testing.T) {
	// EngineRPM=3500 rpm, ThrottlePosition=42%, Lambda1=1.000,
	// ManifoldAirPressure=101.3 kPa (raw 1013), Gear=3.
	data := []byte{0xAC, 0x0D, 42, 0xE8, 0x03, 0xF5, 0x03, 3}
	got, err := DecodeSignals(0x200, data)
	if err != nil {
		t.Fatalf("DecodeSignals: %v", err)
	}
	if !approxEqual(got["EngineRPM"], 3500) {
		t.Errorf("EngineRPM = %v, want 3500", got["EngineRPM"])
	}
	if !approxEqual(got["ThrottlePosition"], 42) {
		t.Errorf("ThrottlePosition = %v, want 42", got["ThrottlePosition"])
	}
	if !approxEqual(got["Lambda1"], 1.0) {
		t.Errorf("Lambda1 = %v, want 1.0", got["Lambda1"])
	}
	if !approxEqual(got["ManifoldAirPressure"], 101.3) {
		t.Errorf("ManifoldAirPressure = %v, want 101.3", got["ManifoldAirPressure"])
	}
	if !approxEqual(got["Gear"], 3) {
		t.Errorf("Gear = %v, want 3", got["Gear"])
	}
}

func TestDecodeSignals_NegativeSigned(t *testing.T) {
	// GForceLateral = -1.234 g -> raw int16 -1234 (0xFB2E) little-endian.
	data := []byte{0, 0, 0x2E, 0xFB, 0, 0, 0, 0}
	got, err := DecodeSignals(0x110, data)
	if err != nil {
		t.Fatalf("DecodeSignals: %v", err)
	}
	if !approxEqual(got["GForceLateral"], -1.234) {
		t.Errorf("GForceLateral = %v, want -1.234", got["GForceLateral"])
	}
}

func TestDecodeSignals_UnknownID(t *testing.T) {
	_, err := DecodeSignals(0x999, []byte{0, 0, 0, 0, 0, 0, 0, 0})
	if err == nil {
		t.Fatal("expected ErrUnknownMessage, got nil")
	}
}

func TestDecodeSignals_ShortPayload(t *testing.T) {
	if _, err := DecodeSignals(0x100, []byte{1, 2}); err == nil {
		t.Fatal("expected error for short payload, got nil")
	}
}
