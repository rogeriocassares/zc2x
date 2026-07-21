"use client"

import { Dispatch, SetStateAction } from "react";

type CanData = Array<{timestamp: Number, data : Number}>

type CanMap = {
  WheelSpeedFL : CanData;
  WheelSpeedFR : CanData;
  WheelSpeedRL : CanData;
  WheelSpeedRR : CanData;
  
  SteeringAngle : CanData;
  GForceLateral : CanData;
  GForceLongitudional : CanData;
  GroundSpeed : CanData;

  BrakePressureFront : CanData;
  BrakePressureRear : CanData;

  EngineRPM : CanData;
  ThrottlePosition : CanData;
  Lambda1 : CanData;
  ManifoldAirPressure : CanData;
  Gear : CanData;

  EngineCoolantTemperature : CanData;
  EngineOilTemperature: CanData;
  ManifoldAirTemperature : CanData;
  EngineOilPressure : CanData;

  FuelLinePressure : CanData;
  FuelUsedRaw : CanData;

  ExhaustCylinderTemperature1: CanData;
  ExhaustCylinderTemperature2: CanData;
  ExhaustCylinderTemperature3: CanData;

  Latitude : CanData;
  Longitude : CanData;

  Altitude : CanData;
  GPSSpeed : CanData;
}


export function canParser(message : Uint8Array, setCanList: Dispatch<SetStateAction<CanMap>>){
  const timestamp = uint8ArrayToNumber(message, 15, 8, true);
  const canId     = uint8ArrayToNumber(message, 23, 8, true);
  const canData   = message.subarray(28, 36);
  console.log(timestamp);
  console.log(canId);
  console.log(canData);
  switch(Number(canId)){
    case 256:
      setCanList(
        prev => {
          prev.WheelSpeedFL.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
          })
          prev.WheelSpeedFR.push({
            timestamp: Number(timestamp),
            data : Number(uint8ArrayToNumber(canData, 2,2, true))
          })
          prev.WheelSpeedRL.push({
            timestamp: Number(timestamp),
            data : Number(uint8ArrayToNumber(canData, 4,2, true))
          })
          prev.WheelSpeedRR.push({
            timestamp : Number(timestamp),
            data : Number(uint8ArrayToNumber(canData, 6,2, true))
          })
          return prev;
        }
      )
      break;
    case 272:
      setCanList(
        prev => {
          prev.SteeringAngle.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
          })
          prev.GForceLateral.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 2,2, true))
          })
          prev.GForceLongitudional.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 4,2, true))
          })
          prev.GroundSpeed.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 6,2, true))
            })
          return prev;
        }
      )
      break;
    case 288:
      setCanList(
        prev => {
          prev.BrakePressureFront.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
          })
          prev.BrakePressureRear.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 2, 2, true))
          })
          return prev;
        }
      )
      break;
    case 512:
      setCanList(
        prev => {
          prev.EngineRPM.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
          })
          prev.ThrottlePosition.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 2,1, true))
          })
          prev.Lambda1.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 3, 2, true))
          })
          prev.ManifoldAirPressure.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 5, 2, true))
          })
          prev.Gear.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 7, 1, true))
          })
          return prev;
        }
      )
      break;
    case 528:
      setCanList(
        prev => {
          prev.EngineCoolantTemperature.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0, 2, true))
          })
          prev.EngineOilTemperature.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 2, 4, true))
          })
          prev.ManifoldAirTemperature.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 4, 6, true))
          })
          prev.EngineOilPressure.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 6, 2, true))
          })
          return prev;
        }
      )
      break;
    case 529:
      setCanList(
        prev => {
          prev.FuelLinePressure.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData,0,2,true))
          })
          prev.FuelUsedRaw.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 2, 4, true))
          })
          return prev;
        }
      )
      break;
    case 544:
      setCanList(
        prev => {
          prev.ExhaustCylinderTemperature1.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0,2,true))
          })
          prev.ExhaustCylinderTemperature2.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 2,2,true))
          })
          prev.ExhaustCylinderTemperature3.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 4,2,true))
          })
          return prev;
        }
      )
      break;
    case 768:
      setCanList(
        prev => {
          prev.Latitude.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0,4,true))
          })
          prev.Longitude.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 4,4,true))
          })
          return prev;
        }
      )
      break;
    case 784:
      setCanList(
        prev => {
          prev.Altitude.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 0,2, true))
          })
          prev.GPSSpeed.push({
            timestamp: Number(timestamp),
            data: Number(uint8ArrayToNumber(canData, 2,4, true))
          })
          return prev;
        }
      )
      break;
  }
}


function uint8ArrayToNumber(byteArray : Uint8Array, offset : number, size : number, bigEndian : boolean) : bigint {
  let result = BigInt(0);
  if(bigEndian){
    for(let i = 0; i < size; i++){
      result = (result << BigInt(8)) | BigInt(byteArray[i + offset]);
    }
  }
  else{
    for(let i = 0; i < size; i++){
      result = (result >> BigInt(8)) | (BigInt(byteArray[i + offset]) << BigInt(i * 8));
    }
  }
  return result;
}
