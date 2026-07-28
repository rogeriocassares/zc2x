"use client"

import { use, useEffect, useState } from "react"
import { LineCanChart } from "./lineChart";
import { getNatsConnection } from "@/lib/nats";
import { CanMap, CanMapInit, canParser } from "@/lib/can";
import { NatsConnection } from "@nats-io/nats-core";



export function Chart(){
  const [canMap, setCanMap] = useState<CanMap>(CanMapInit);
  useEffect(
    () => {
      let nc : NatsConnection | null;
      async function getData(){
        nc = await getNatsConnection();
        if(nc == null){
          return;
        }
        nc.subscribe('zc2x.can.rsu', {
          callback: (err, msg) => {
            canParser(msg.data, setCanMap);
          }
        });
      }
      getData();
      return () =>{
        nc?.close();
      }
    },[]);
  return (
    <div className="w-[100%]">
      <div className="flex">
        <LineCanChart title="Brake Pressure Front" data={canMap?.BrakePressureFront} unit="kPa"/>
        <LineCanChart title="Brake Pressure Rear" data={canMap?.BrakePressureRear} unit="kPa"/>
      </div>
      <div className="flex">
        <LineCanChart title="Engine Oil Pressure" data={canMap?.EngineOilPressure}    unit="kPa"/>
        <LineCanChart title="Engine Oil Temperature" data={canMap?.EngineOilTemperature} unit="°C"/>
      </div>
      <div className="flex">
        <LineCanChart title="Altitude" data={canMap?.Altitude} unit="m"/>
        <LineCanChart title="Latitude" data={canMap?.Latitude} unit="m"/>
        <LineCanChart title="Longitude" data={canMap?.Longitude} unit="m"/>
        <LineCanChart title="GPS Speed" data={canMap?.GPSSpeed} unit="km/h"/>
      </div>
      <div className="flex">
        <LineCanChart title="Wheel Speed FL" data={canMap?.WheelSpeedFL} unit="km/h"/>
        <LineCanChart title="Wheel Speed FR" data={canMap?.WheelSpeedFR} unit="km/h"/>
        <LineCanChart title="Wheel Speed RL" data={canMap?.WheelSpeedRL} unit="km/h"/>
        <LineCanChart title="Wheel Speed RR" data={canMap?.WheelSpeedRR} unit="km/h"/>
      </div>
      <div className="flex">
        <LineCanChart title="SteeringAngle" data={canMap?.SteeringAngle} unit="º "/>
        <LineCanChart title="G Force Lat" data={canMap?.GForceLateral} unit="G"/>
        <LineCanChart title="G Force Long" data={canMap?.GForceLongitudional} unit="G"/>
        <LineCanChart title="G Force Vert" data={canMap?.GForceVert} unit="G"/>
      </div>
    </div>
  );
}
