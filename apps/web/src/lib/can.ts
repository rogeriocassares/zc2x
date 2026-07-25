"use client"

import { Dispatch, SetStateAction } from "react";

export type CanData = Array<{ timestamp: number, data: number }>

export type CanMap = {
    WheelSpeedFL: CanData;
    WheelSpeedFR: CanData;
    WheelSpeedRL: CanData;
    WheelSpeedRR: CanData;

    SteeringAngle: CanData;
    GForceLateral: CanData;
    GForceLongitudional: CanData;
    GroundSpeed: CanData;

    BrakePressureFront: CanData;
    BrakePressureRear: CanData;

    EngineRPM: CanData;
    ThrottlePosition: CanData;
    Lambda1: CanData;
    ManifoldAirPressure: CanData;
    Gear: CanData;

    EngineCoolantTemperature: CanData;
    EngineOilTemperature: CanData;
    ManifoldAirTemperature: CanData;
    EngineOilPressure: CanData;

    FuelLinePressure: CanData;
    FuelUsedRaw: CanData;

    ExhaustCylinderTemperature1: CanData;
    ExhaustCylinderTemperature2: CanData;
    ExhaustCylinderTemperature3: CanData;

    Latitude: CanData;
    Longitude: CanData;

    Altitude: CanData;
    GPSSpeed: CanData;
}

export let CanMapInit: CanMap = {
    WheelSpeedFL: [],
    WheelSpeedFR: [],
    WheelSpeedRL: [],
    WheelSpeedRR: [],

    SteeringAngle: [],
    GForceLateral: [],
    GForceLongitudional: [],
    GroundSpeed: [],

    BrakePressureFront: [],
    BrakePressureRear: [],

    EngineRPM: [],
    ThrottlePosition: [],
    Lambda1: [],
    ManifoldAirPressure: [],
    Gear: [],

    EngineCoolantTemperature: [],
    EngineOilTemperature: [],
    ManifoldAirTemperature: [],
    EngineOilPressure: [],

    FuelLinePressure: [],
    FuelUsedRaw: [],

    ExhaustCylinderTemperature1: [],
    ExhaustCylinderTemperature2: [],
    ExhaustCylinderTemperature3: [],

    Latitude: [],
    Longitude: [],

    Altitude: [],
    GPSSpeed: [],
}

const NATS_PACKET_SIZE = 34;
let timestamp =0;
export function canParser(message: Uint8Array, setCanList: Dispatch<SetStateAction<CanMap>>) {
  const messageView = new DataView(message.buffer);
    for (let i = 0; i < message.length / NATS_PACKET_SIZE; i++) {
        const timestamp = messageView.getBigInt64(12 + NATS_PACKET_SIZE * i);
        const canIdArray = message.subarray(19 + NATS_PACKET_SIZE * i, 23 + NATS_PACKET_SIZE * i).toReversed();
        const canId = canIdArray[2] << 8 | canIdArray[3];
      const canData = message.subarray(24 + NATS_PACKET_SIZE * i, 32 + NATS_PACKET_SIZE * i);
        switch (Number(canId)) {
            case 256:
                setCanList(
                    prev => {
                        prev.WheelSpeedFL = [...prev.WheelSpeedFL, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
                        }]
                        prev.WheelSpeedFR = [...prev.WheelSpeedFR, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 2, 2, true))
                        }]
                        prev.WheelSpeedRL = [...prev.WheelSpeedRL, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 4, 2, true))
                        }]
                        prev.WheelSpeedRR = [...prev.WheelSpeedRR, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 6, 2, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 272:
                setCanList(
                    prev => {
                        prev.SteeringAngle = [...prev.SteeringAngle, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
                        }]
                        prev.GForceLateral = [...prev.GForceLateral, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 2, 2, true))
                        }]
                        prev.GForceLongitudional = [...prev.GForceLongitudional, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 4, 2, true))
                        }]
                        prev.GroundSpeed = [...prev.GroundSpeed, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 6, 2, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 288:
                setCanList(
                    prev => {
                        prev.BrakePressureFront = [...prev.BrakePressureFront, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
                        }]
                        prev.BrakePressureRear = [...prev.BrakePressureRear, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 2, 2, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 512:
                setCanList(
                    prev => {
                        prev.EngineRPM = [...prev.EngineRPM, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 2, true)) / 6
                        }]
                        prev.ThrottlePosition = [...prev.ThrottlePosition, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 2, 1, true))
                        }]
                        prev.Lambda1 = [...prev.Lambda1, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 3, 2, true))
                        }]
                        prev.ManifoldAirPressure = [...prev.ManifoldAirPressure, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 5, 2, true))
                        }]
                        prev.Gear = [...prev.Gear, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 7, 1, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 528:
                setCanList(
                    prev => {
                        prev.EngineCoolantTemperature = [...prev.EngineCoolantTemperature, {
                            timestamp: Number(timestamp),
                            data: (Number(uint8ArrayToNumber(canData, 0, 2, true)) - 400) * 10
                        }]
                        prev.EngineOilTemperature = [...prev.EngineOilTemperature, {
                            timestamp: Number(timestamp),
                            data: (Number(uint8ArrayToNumber(canData, 2, 2, true)) - 400) * 10
                        }]
                        prev.ManifoldAirTemperature = [...prev.ManifoldAirTemperature, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 4, 2, true))
                        }]
                        prev.EngineOilPressure = [...prev.EngineOilPressure, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 6, 2, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 529:
                setCanList(
                    prev => {
                        prev.FuelLinePressure = [...prev.FuelLinePressure, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
                        }]
                        prev.FuelUsedRaw = [...prev.FuelUsedRaw, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 2, 4, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 544:
                setCanList(
                    prev => {
                        prev.ExhaustCylinderTemperature1 = [...prev.ExhaustCylinderTemperature1, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
                        }]
                        prev.ExhaustCylinderTemperature2 = [...prev.ExhaustCylinderTemperature2, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 2, 2, true))
                        }]
                        prev.ExhaustCylinderTemperature3 = [...prev.ExhaustCylinderTemperature3, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 4, 2, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 768:
                setCanList(
                    prev => {
                        prev.Latitude = [...prev.Latitude, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 4, true))
                        }]
                        prev.Longitude = [...prev.Longitude, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 4, 4, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
            case 784:
                setCanList(
                    prev => {
                        prev.Altitude = [...prev.Altitude, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
                        }]
                        prev.GPSSpeed = [...prev.GPSSpeed, {
                            timestamp: Number(timestamp),
                            data: Number(uint8ArrayToNumber(canData, 2, 4, true))
                        }]
                        return { ...prev };
                    }
                )
                break;
        }
    }
}


function uint8ArrayToNumber(byteArray: Uint8Array, offset: number, size: number, bigEndian: boolean): bigint {
    let result = BigInt(0);
    for (let i = 0; i < size; i++) {
        result = result | BigInt(byteArray[offset + i]);
        if (i == size - 1) break;
        result = result << BigInt(8);
    }
    return result;
}
