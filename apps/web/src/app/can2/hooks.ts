"use client";

import { useCallback, useSyncExternalStore } from "react";
import { telemetryStore } from "./telemetry-store";
import type {
  Catalog,
  ConnectionStatus,
  DeviceList,
  MessageSnapshot,
} from "./telemetry-store";
import { useSelectedDevice } from "./use-selection";

// useSyncExternalStore, not useEffect+useState: telemetryStore is mutable
// state that lives outside React (a NATS subscription accumulating live
// messages), which is exactly the case this hook exists for -- it's the
// primitive React ships specifically to subscribe a component to an
// external store without the tearing/inconsistent-snapshot issues a manual
// useEffect subscription can hit under concurrent rendering.
export function useConnectionStatus(): ConnectionStatus {
  return useSyncExternalStore(
    telemetryStore.subscribeStatus,
    telemetryStore.getStatus,
    telemetryStore.getServerStatus
  );
}

// The full set of CAN messages/signals ever observed (see telemetry-store's
// Catalog doc) -- what the message-picker menu lists from. Only changes
// when a genuinely new message or signal is first seen, so this is a rare
// re-render, not a per-value one.
export function useCatalog(): Catalog {
  return useSyncExternalStore(
    telemetryStore.subscribeCatalog,
    telemetryStore.getCatalogSnapshot,
    telemetryStore.getServerCatalogSnapshot
  );
}

// The list of devices seen so far -- rare re-render, same reasoning as
// useCatalog.
export function useDeviceList(): DeviceList {
  return useSyncExternalStore(
    telemetryStore.subscribeDeviceList,
    telemetryStore.getDeviceListSnapshot,
    telemetryStore.getServerDeviceListSnapshot
  );
}

// One CAN message's live signal values for one (device_id, origin) pair.
// This is the hook that actually fires often (every time that message
// arrives) -- scoping it to exactly one (devKey, canMessageName) topic is
// what keeps a value update from re-rendering every other card on the
// dashboard.
export function useMessageState(
  devKey: string | null,
  canMessageName: string
): MessageSnapshot {
  const subscribe = useCallback(
    (listener: () => void) => {
      if (!devKey) return () => {};
      return telemetryStore.subscribeMessage(devKey, canMessageName, listener);
    },
    [devKey, canMessageName]
  );
  const getSnapshot = useCallback(
    () => (devKey ? telemetryStore.getMessageSnapshot(devKey, canMessageName) : undefined),
    [devKey, canMessageName]
  );
  const getServerSnapshot = useCallback((): MessageSnapshot => undefined, []);
  return useSyncExternalStore(subscribe, getSnapshot, getServerSnapshot);
}

// Resolves the persisted device selection against who's actually online:
// falls back to the first known device when nothing's been picked yet, or
// the persisted pick is no longer among known devices, without persisting
// that fallback as a real choice. Shared by Sidebar (to show the right
// value in the <select>) and TelemetryDashboard (to know which device's
// data to render) -- they're siblings now (see layout.tsx), so both need
// the same resolved value rather than each guessing independently.
//
// Returns the compound (device_id, origin) key (see telemetry-store.ts's
// deviceKey), not a raw device_id -- RSU relays OBU's packet bytes
// unmodified (RFC-0001), so device_id alone can't distinguish OBU's direct
// WiFi link from OBU's data arriving via RSU's XBee relay, which is exactly
// what selecting between them requires.
export function useActiveDeviceKey(): string | null {
  const deviceList = useDeviceList();
  const [selectedKey] = useSelectedDevice();
  return selectedKey && deviceList.some((d) => d.key === selectedKey)
    ? selectedKey
    : (deviceList[0]?.key ?? null);
}
