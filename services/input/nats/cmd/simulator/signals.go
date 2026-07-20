// Command simulator's synthetic-signal generation. See main.go's package
// doc for what this simulator is and isn't modeling.
package main

import (
	"encoding/binary"
	"hash/fnv"
	"math"

	"github.com/rogeriocassares/zc2x/services/input/nats/internal/candb"
)

// rawRange returns the raw-value domain candb.Signal.Type can represent.
func rawRange(t candb.ValueType) (min, max int64) {
	switch t {
	case candb.Uint8:
		return 0, 255
	case candb.Int16:
		return -32768, 32767
	case candb.Uint16:
		return 0, 65535
	case candb.Uint32:
		return 0, 4294967295
	case candb.Int32:
		return -2147483648, 2147483647
	case candb.Bit:
		return 0, 1
	default:
		return 0, 0
	}
}

// int32Amplitude caps the synthetic swing for Int32 signals (Latitude,
// Longitude, FuelUsedRaw — see candb.go's Signal doc) well below their full
// 32-bit domain. Those three signals are Bias=0, scale-only encodings whose
// real physical range is much narrower than +/-2^31 (e.g. degrees, not the
// ~214 degrees a full-range sine would produce for a 1e-7 deg/LSB signal);
// this keeps generated values plausible without needing a per-signal
// physical-range table the Go candb mirror doesn't otherwise carry.
const int32Amplitude = 2_000_000

// syntheticRaw deterministically derives a raw signal value from (seed,
// canID, signal, elapsed-seconds-since-start): a smooth sine wave whose
// frequency and phase are hashed from seed+canID+signal name, so different
// devices and signals drift independently without any device needing to
// share mutable state. This is a pipeline exerciser, not a vehicle-dynamics
// model — it doesn't honor each signal's documented physical range (unlike
// the physically-modeled ecu/ hardware firmware simulator), only its raw
// wire-encoding domain, so values may occasionally look physically
// implausible (e.g. Gear outside 1-6). Two publishers given the same seed
// (an OBU and the RSU relaying it — see main.go) evaluate this identically
// at the same elapsed time, approximating a live relay without piping bytes
// between goroutines.
func syntheticRaw(seed uint64, canID uint32, sig candb.Signal, elapsedSeconds float64) int64 {
	min, max := rawRange(sig.Type)

	h := fnv.New64a()
	h.Write([]byte{byte(canID), byte(canID >> 8), byte(canID >> 16), byte(canID >> 24)})
	h.Write([]byte(sig.Name))
	var seedBuf [8]byte
	binary.LittleEndian.PutUint64(seedBuf[:], seed)
	h.Write(seedBuf[:])
	mixed := h.Sum64()

	if sig.Type == candb.Bit {
		freq := 0.02 + 0.08*fracOf(mixed) // slow, occasional flips
		if math.Sin(2*math.Pi*freq*elapsedSeconds+phaseOf(mixed)) > 0 {
			return 1
		}
		return 0
	}

	amplitude := float64(max-min) * 0.35
	if sig.Type == candb.Int32 {
		amplitude = int32Amplitude
	}
	center := float64(min+max) / 2
	freq := 0.05 + 0.15*fracOf(mixed)
	v := center + amplitude*math.Sin(2*math.Pi*freq*elapsedSeconds+phaseOf(mixed))
	if v < float64(min) {
		v = float64(min)
	}
	if v > float64(max) {
		v = float64(max)
	}
	return int64(v)
}

// fracOf/phaseOf derive a [0,1) fraction / a [0,2*pi) phase from a hash, so
// each signal gets its own frequency and starting phase without persisted
// per-signal state.
func fracOf(h uint64) float64 {
	return float64(h%1_000_000) / 1_000_000
}

func phaseOf(h uint64) float64 {
	return fracOf(h>>20) * 2 * math.Pi
}

// encodeSignalRaw packs raw into buf at sig's offset, mirroring
// candb.Signal.decodeRaw's layout in reverse (little-endian, byte- or
// bit-aligned per sig.Type). buf must be at least as long as the owning
// message's DLC.
func encodeSignalRaw(buf []byte, sig candb.Signal, raw int64) {
	if sig.Type == candb.Bit {
		byteIdx, bitIdx := sig.BitOffset/8, uint(sig.BitOffset%8)
		if raw != 0 {
			buf[byteIdx] |= 1 << bitIdx
		}
		return
	}
	switch sig.Type {
	case candb.Uint8:
		buf[sig.ByteOffset] = byte(raw)
	case candb.Int16, candb.Uint16:
		binary.LittleEndian.PutUint16(buf[sig.ByteOffset:], uint16(raw))
	case candb.Uint32, candb.Int32:
		binary.LittleEndian.PutUint32(buf[sig.ByteOffset:], uint32(raw))
	}
}
