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
    <div>
      <LineCanChart title="Brake Pressure Front" data={canMap?.BrakePressureFront} unit="kPa"/>
      <LineCanChart title="Brake Pressure Rear" data={canMap?.BrakePressureRear} unit="kPa"/>
    </div>
  );
}
