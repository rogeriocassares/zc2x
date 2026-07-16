// Package internal — signal decoding. See candb for the CAN2 message/signal
// database (mirrors docs/architecture/zc2x-can2.dbc). This file decodes
// each validated CAN_FRAME packet's payload into named, scaled
// engineering-unit signals and republishes them as JSON to a per-message
// JetStream subject, additionally to (never instead of) the raw packet
// republish in adapter.go's handleMessage. Unknown CAN IDs are expected
// (OBU/RSU forward every CAN ID unfiltered) and are silently skipped.
package internal

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"strings"

	"github.com/rogeriocassares/zc2x/services/input/nats/internal/candb"
)

// SignalPublishSubjectFn maps a decoded message name and its source NATS
// Core subject to the JetStream subject its signals should be published on.
type SignalPublishSubjectFn func(messageName string, sourceSubject string) string

// DefaultSignalPublishSubject publishes decoded signals for message name
// "PE1" arriving on sourceSubject "zc2x.can.obu" to e.g.
// signalPrefix+"obu.PE1" (so "zc2x.js.signals." -> "zc2x.js.signals.obu.PE1").
//
// The source token here is the relay path (which NATS Core subject the
// packet arrived on — OBU's direct WiFi link vs RSU's XBee relay), not the
// originating device: RSU forwards packet bytes unchanged (RFC-0001
// immutability), so a packet's DeviceID is always OBU's, even when RSU
// relayed it. The relay path is only recoverable from sourceSubject, which
// is why it's threaded through here rather than read back out of the
// decoded payload. Including it as a leading subject token (rather than
// only a JSON field) lets consumers filter by it natively — e.g.
// "zc2x.js.signals.*.PE1" for PE1 from any path, or "zc2x.js.signals.obu.>"
// for everything that arrived via OBU — without inspecting every payload.
func DefaultSignalPublishSubject(signalPrefix, sourcePrefix string) SignalPublishSubjectFn {
	return func(messageName, sourceSubject string) string {
		source := strings.TrimPrefix(sourceSubject, sourcePrefix)
		return signalPrefix + source + "." + messageName
	}
}

// SignalMessage is the JSON payload published per decoded CAN message.
type SignalMessage struct {
	MessageName string             `json:"message"`
	Source      string             `json:"source"`
	CANID       uint32             `json:"can_id"`
	DeviceID    string             `json:"device_id"`
	Sequence    uint32             `json:"sequence"`
	Timestamp   uint64             `json:"timestamp"`
	Signals     map[string]float64 `json:"signals"`
}

// decodeAndPublishSignals decodes pkt's CAN payload via candb and, if
// pkt.CANID is a known ZC2X CAN2 message, republishes the decoded signals
// as JSON. candb.ErrUnknownMessage is expected and not logged; any other
// decode error is logged and dropped (best-effort — never blocks or fails
// the raw-packet republish that already happened in handleMessage).
func (a *Adapter) decodeAndPublishSignals(ctx context.Context, pkt Packet, sourceSubject string) {
	signals, err := candb.DecodeSignals(pkt.CANID, pkt.Data[:pkt.DLC])
	if err != nil {
		if errors.Is(err, candb.ErrUnknownMessage) {
			return
		}
		log.Printf("internal: signal decode failed for can_id=0x%03x on %q: %v", pkt.CANID, sourceSubject, err)
		return
	}

	msg, _ := candb.MessageByID(pkt.CANID) // ok guaranteed: DecodeSignals just succeeded for this ID

	payload, err := json.Marshal(SignalMessage{
		MessageName: msg.Name,
		Source:      sourceSubject,
		CANID:       pkt.CANID,
		DeviceID:    fmt.Sprintf("%x", pkt.DeviceID),
		Sequence:    pkt.Sequence,
		Timestamp:   pkt.Timestamp,
		Signals:     signals,
	})
	if err != nil {
		log.Printf("internal: marshal signals for %s failed: %v", msg.Name, err)
		return
	}

	subject := a.cfg.SignalPublishSubjectFn(msg.Name, sourceSubject)
	if _, err := a.js.Publish(ctx, subject, payload); err != nil {
		log.Printf("internal: publish signals to jetstream %q failed: %v", subject, err)
	}
}
