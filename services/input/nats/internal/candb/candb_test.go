package candb

import (
	"encoding/binary"
	"math"
	"testing"
)

const epsilon = 1e-9

func approxEqual(a, b float64) bool {
	return math.Abs(a-b) < epsilon
}

func TestDecodeSignals_PE1(t *testing.T) {
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

func TestDecodeSignals_GPS1_Int32(t *testing.T) {
	// Latitude = -23.5505 deg, Longitude = -46.6333 deg, at 1e-7 deg/LSB.
	// var, not const: forces runtime evaluation below, since Go disallows
	// converting an out-of-range *constant* expression straight to uint32
	// even though the int32->uint32 bit-reinterpretation is fine at runtime.
	lat, lon := -23.5505, -46.6333
	data := make([]byte, 8)
	binary.LittleEndian.PutUint32(data[0:], uint32(int32(lat/0.0000001)))
	binary.LittleEndian.PutUint32(data[4:], uint32(int32(lon/0.0000001)))

	got, err := DecodeSignals(0x300, data)
	if err != nil {
		t.Fatalf("DecodeSignals: %v", err)
	}
	// int32 @ 1e-7 deg/LSB can't round-trip a float64 division exactly;
	// 1e-6 deg (~11cm) is well within the resolution this signal claims.
	const tolerance = 1e-6
	if math.Abs(got["Latitude"]-lat) > tolerance {
		t.Errorf("Latitude = %v, want %v", got["Latitude"], lat)
	}
	if math.Abs(got["Longitude"]-lon) > tolerance {
		t.Errorf("Longitude = %v, want %v", got["Longitude"], lon)
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
