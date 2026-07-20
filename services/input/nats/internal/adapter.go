// Package internal implements the NATS Core -> JetStream bridge for ZC2X.
//
// OBU and RSU publish zc2x_packet_t wire packets (see
// framework/core/packet/README.md and docs/ARCHITECTURE.md) to NATS Core
// subjects as a fire-and-forget, non-durable transport. This package
// subscribes to that traffic, decodes and validates each packet, and
// republishes only packets that pass validation to a JetStream stream for
// durable storage. Per RFC-0001 (packets are immutable while traveling
// through the system), decoding here is for validation and subject routing
// only — the bytes forwarded to JetStream are the original packet, unchanged.
//
// signals.go additionally decodes each CAN_FRAME packet's payload into
// named, scaled engineering-unit signals (see internal/candb for the CAN2
// message/signal database) and republishes each one as its own JSON
// TelemetryRecord to a separate stream, consumable directly by the apps/web
// Next.js dashboard (or any other JSON-capable subscriber). That's purely
// additive — it never changes what's forwarded raw.
package internal

import (
	"context"
	"encoding/binary"
	"fmt"
	"log"
	"strings"

	"github.com/nats-io/nats.go"
	"github.com/nats-io/nats.go/jetstream"
)

const (
	// PacketSize is the wire size of zc2x_packet_t (framework/core/packet):
	// type(1) + device_id(6) + sequence(4) + timestamp(8) + can_id(4) +
	// dlc(1) + data(8) + crc16(2), with no padding (the C struct is
	// __attribute__((packed))).
	PacketSize = 34

	// DeviceIDSize matches ZC2X_DEVICE_ID_SIZE in zc2x_packet.h.
	DeviceIDSize = 6

	// CANDataSize matches ZC2X_CAN_DATA_SIZE in zc2x_packet.h.
	CANDataSize = 8

	crc16Poly = 0x1021
	crc16Init = 0xFFFF
)

// PacketType mirrors zc2x_packet_type_t.
type PacketType uint8

// Packet types defined in zc2x_packet.h. Only CANFrame is produced by the
// current firmware; the rest are forward-looking.
const (
	PacketTypeInvalid    PacketType = 0x00
	PacketTypeCANFrame   PacketType = 0x01
	PacketTypeGNSS       PacketType = 0x02
	PacketTypeSysEvt     PacketType = 0x03
	PacketTypeDiagnostic PacketType = 0x04
	PacketTypeOTA        PacketType = 0x05
	PacketTypeLog        PacketType = 0x06
)

// Packet is the decoded form of zc2x_packet_t.
type Packet struct {
	Type      PacketType
	DeviceID  [DeviceIDSize]byte
	Sequence  uint32
	Timestamp uint64
	CANID     uint32
	DLC       uint8
	Data      [CANDataSize]byte
	CRC16     uint16
}

// DecodePacket parses and validates a zc2x_packet_t wire payload. All
// multibyte fields are little-endian (RFC-0001 §8), matching the native
// byte order of the esp32c6/esp32s3 firmware that produces them.
//
// It returns an error if the payload isn't PacketSize bytes, the type isn't
// CANFrame, dlc exceeds CANDataSize, or the CRC-16 doesn't match.
func DecodePacket(raw []byte) (Packet, error) {
	var p Packet
	if len(raw) != PacketSize {
		return p, fmt.Errorf("adapter: invalid packet size %d, want %d", len(raw), PacketSize)
	}

	p.Type = PacketType(raw[0])
	copy(p.DeviceID[:], raw[1:7])
	p.Sequence = binary.LittleEndian.Uint32(raw[7:11])
	p.Timestamp = binary.LittleEndian.Uint64(raw[11:19])
	p.CANID = binary.LittleEndian.Uint32(raw[19:23])
	p.DLC = raw[23]
	copy(p.Data[:], raw[24:32])
	p.CRC16 = binary.LittleEndian.Uint16(raw[32:34])

	if p.Type != PacketTypeCANFrame {
		return p, fmt.Errorf("adapter: unsupported packet type 0x%02x", uint8(p.Type))
	}
	if p.DLC > CANDataSize {
		return p, fmt.Errorf("adapter: dlc %d exceeds max %d", p.DLC, CANDataSize)
	}

	if expected := crc16CCITTFalse(raw[:32]); expected != p.CRC16 {
		return p, fmt.Errorf("adapter: crc mismatch: got 0x%04x want 0x%04x", p.CRC16, expected)
	}

	return p, nil
}

// crc16CCITTFalse mirrors framework/core/packet/src/packet_crc.c bit-for-bit:
// poly 0x1021, init 0xFFFF, no input/output reflection, no final XOR.
func crc16CCITTFalse(data []byte) uint16 {
	crc := uint16(crc16Init)
	for _, b := range data {
		crc ^= uint16(b) << 8
		for i := 0; i < 8; i++ {
			if crc&0x8000 != 0 {
				crc = (crc << 1) ^ crc16Poly
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

// PublishSubjectFn maps a decoded packet and its source NATS Core subject to
// the JetStream subject it should be republished on.
type PublishSubjectFn func(p Packet, sourceSubject string) string

// DefaultPublishSubject rewrites a source subject under sourcePrefix (e.g.
// "zc2x.can.") to the same suffix under targetPrefix (e.g. "zc2x.js.can."),
// so "zc2x.can.obu" becomes "zc2x.js.can.obu".
func DefaultPublishSubject(sourcePrefix, targetPrefix string) PublishSubjectFn {
	return func(_ Packet, sourceSubject string) string {
		return targetPrefix + strings.TrimPrefix(sourceSubject, sourcePrefix)
	}
}

// Config holds the adapter's runtime configuration.
type Config struct {
	// NATSURL is the NATS Core server URL, e.g. "nats://127.0.0.1:4222".
	NATSURL string
	// SourceSubject is the NATS Core subject to subscribe to, e.g. "zc2x.can.*".
	SourceSubject string
	// StreamName is the JetStream stream to create/publish into.
	StreamName string
	// StreamSubjects is the subject filter the stream captures.
	StreamSubjects []string
	// PublishSubjectFn maps a decoded packet to its JetStream publish subject.
	PublishSubjectFn PublishSubjectFn
	// SourcePrefix is trimmed from a packet's source NATS Core subject to
	// derive the "origin" tag on published telemetry (see signals.go). Must
	// match whatever prefix SourceSubject/PublishSubjectFn use.
	SourcePrefix string

	// TelemetryStreamName is the JetStream stream decoded per-signal Line
	// Protocol telemetry (see signals.go) is published into. Separate from
	// StreamName so its retention/lifecycle can be tuned independently —
	// this is additive data, not a replacement for the raw stream above.
	TelemetryStreamName string
	// TelemetryStreamSubjects is the subject filter the telemetry stream captures.
	TelemetryStreamSubjects []string
	// TelemetryPublishSubjectFn maps a packet's source subject to its
	// telemetry JetStream publish subject.
	TelemetryPublishSubjectFn TelemetryPublishSubjectFn
}

// Adapter bridges NATS Core to JetStream.
type Adapter struct {
	cfg Config
	nc  *nats.Conn
	js  jetstream.JetStream
}

// NewAdapter connects to NATS Core and initializes the JetStream context.
func NewAdapter(cfg Config) (*Adapter, error) {
	nc, err := nats.Connect(cfg.NATSURL, nats.Name("zc2x-input-nats"))
	if err != nil {
		return nil, fmt.Errorf("adapter: connect to nats at %s: %w", cfg.NATSURL, err)
	}

	js, err := jetstream.New(nc)
	if err != nil {
		nc.Close()
		return nil, fmt.Errorf("adapter: init jetstream: %w", err)
	}

	return &Adapter{cfg: cfg, nc: nc, js: js}, nil
}

// EnsureStream creates the raw-packet and decoded-telemetry JetStream
// streams if they don't exist, or updates their subject filters if they do.
func (a *Adapter) EnsureStream(ctx context.Context) error {
	if err := a.ensureStream(ctx, a.cfg.StreamName, a.cfg.StreamSubjects); err != nil {
		return err
	}
	return a.ensureStream(ctx, a.cfg.TelemetryStreamName, a.cfg.TelemetryStreamSubjects)
}

func (a *Adapter) ensureStream(ctx context.Context, name string, subjects []string) error {
	_, err := a.js.CreateOrUpdateStream(ctx, jetstream.StreamConfig{
		Name:     name,
		Subjects: subjects,
		Storage:  jetstream.FileStorage,
	})
	if err != nil {
		return fmt.Errorf("adapter: ensure stream %s: %w", name, err)
	}
	return nil
}

// Run subscribes to SourceSubject on NATS Core and blocks, decoding and
// republishing validated packets to JetStream, until ctx is canceled.
func (a *Adapter) Run(ctx context.Context) error {
	msgs := make(chan *nats.Msg, 64)
	sub, err := a.nc.ChanSubscribe(a.cfg.SourceSubject, msgs)
	if err != nil {
		return fmt.Errorf("adapter: subscribe %s: %w", a.cfg.SourceSubject, err)
	}
	defer sub.Unsubscribe()

	log.Printf("adapter: listening on %q, forwarding valid packets to stream %q",
		a.cfg.SourceSubject, a.cfg.StreamName)

	for {
		select {
		case <-ctx.Done():
			return nil
		case msg := <-msgs:
			a.handleMessage(ctx, msg)
		}
	}
}

// handleMessage decodes and republishes one NATS Core message, which may be
// a single packet or a batch of them: OBU/RSU's nats_task accumulates up to
// N packets before publishing (see OBU_NATS_BATCH_MAX_PACKETS /
// RSU_NATS_BATCH_MAX_PACKETS in device_config.h) to cut per-publish
// protocol/TCP-segment overhead, so the payload here is always a multiple
// of PacketSize — one packet concatenated after another, each still
// independently CRC-validated. This is fully backward compatible with an
// unbatched publisher, which is just the N=1 case. Each packet is
// republished to JetStream individually, at its own packet granularity,
// regardless of how many arrived batched in the same NATS Core message.
func (a *Adapter) handleMessage(ctx context.Context, msg *nats.Msg) {
	if len(msg.Data) == 0 || len(msg.Data)%PacketSize != 0 {
		log.Printf("adapter: dropping message on %q: invalid size %d (not a positive multiple of %d)",
			msg.Subject, len(msg.Data), PacketSize)
		return
	}

	for offset := 0; offset < len(msg.Data); offset += PacketSize {
		raw := msg.Data[offset : offset+PacketSize]

		pkt, err := DecodePacket(raw)
		if err != nil {
			log.Printf("adapter: dropping packet at offset %d on %q: %v", offset, msg.Subject, err)
			continue
		}

		subject := a.cfg.PublishSubjectFn(pkt, msg.Subject)
		if _, err := a.js.Publish(ctx, subject, raw); err != nil {
			log.Printf("adapter: publish to jetstream %q failed: %v", subject, err)
			continue
		}

		log.Printf("adapter: %s -> %s seq=%d device=%x can_id=0x%03x dlc=%d",
			msg.Subject, subject, pkt.Sequence, pkt.DeviceID, pkt.CANID, pkt.DLC)

		a.decodeAndPublishSignals(ctx, pkt, msg.Subject)
	}
}

// Close drains and closes the NATS connection.
func (a *Adapter) Close() {
	if a.nc != nil {
		a.nc.Close()
	}
}
