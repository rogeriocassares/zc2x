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

	"github.com/rogeriocassares/zc2x/services/input/nats/internal/candb"
)

// SignalPublishSubjectFn maps a decoded message name to the JetStream
// subject its signals should be published on.
type SignalPublishSubjectFn func(messageName string) string

// DefaultSignalPublishSubject publishes decoded signals for message name
// "EngineCore" to e.g. prefix+"EngineCore" (so "zc2x.js.signals." ->
// "zc2x.js.signals.EngineCore").
func DefaultSignalPublishSubject(prefix string) SignalPublishSubjectFn {
	return func(messageName string) string {
		return prefix + messageName
	}
}

// SignalMessage is the JSON payload published per decoded CAN message.
type SignalMessage struct {
	MessageName string             `json:"message"`
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

	subject := a.cfg.SignalPublishSubjectFn(msg.Name)
	if _, err := a.js.Publish(ctx, subject, payload); err != nil {
		log.Printf("internal: publish signals to jetstream %q failed: %v", subject, err)
	}
}
