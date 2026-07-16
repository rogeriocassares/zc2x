// Command nats is the ZC2X NATS Core -> JetStream input service. It
// subscribes to the raw packet subjects OBU/RSU publish to on NATS Core,
// decodes and validates each zc2x_packet_t, and republishes valid packets to
// a durable JetStream stream.
package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"

	"github.com/nats-io/nats.go"

	"github.com/rogeriocassares/zc2x/services/input/nats/internal"
)

func main() {
	sourcePrefix := getenv("NATS_SOURCE_PREFIX", "zc2x.can.")
	targetPrefix := getenv("JETSTREAM_SUBJECT_PREFIX", "zc2x.js.can.")
	signalPrefix := getenv("NATS_SIGNAL_SUBJECT_PREFIX", "zc2x.js.signals.")

	cfg := internal.Config{
		NATSURL:          getenv("NATS_URL", nats.DefaultURL),
		SourceSubject:    getenv("NATS_SOURCE_SUBJECT", sourcePrefix+"*"),
		StreamName:       getenv("JETSTREAM_STREAM", "ZC2X_CAN"),
		StreamSubjects:   []string{targetPrefix + ">"},
		PublishSubjectFn: internal.DefaultPublishSubject(sourcePrefix, targetPrefix),

		SignalStreamName:       getenv("JETSTREAM_SIGNAL_STREAM", "ZC2X_SIGNALS"),
		SignalStreamSubjects:   []string{signalPrefix + ">"},
		SignalPublishSubjectFn: internal.DefaultSignalPublishSubject(signalPrefix, sourcePrefix),
	}

	adapter, err := internal.NewAdapter(cfg)
	if err != nil {
		log.Fatalf("main: %v", err)
	}
	defer adapter.Close()

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	if err := adapter.EnsureStream(ctx); err != nil {
		log.Fatalf("main: %v", err)
	}

	if err := adapter.Run(ctx); err != nil {
		log.Fatalf("main: %v", err)
	}
}

func getenv(key, fallback string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return fallback
}
