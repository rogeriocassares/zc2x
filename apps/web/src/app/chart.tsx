"use client"

import { use, useEffect, useState } from "react"
import { LineCanChart } from "./lineChart";
import { getNatsConnection } from "@/lib/nats";
import { canParser } from "@/lib/can";
import { NatsConnection } from "@nats-io/nats-core";



export function Chart(){
  const [canMap, setCanMap] = useState<Map<string, []>>(new Map());
  useEffect(
    () => {
      let nc : NatsConnection | null;
      async function getData(){
        nc = await getNatsConnection();
        if(nc == null){
          return;
        }
        nc.subscribe('zc2x.can', {
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
      <LineCanChart title="Temperatura pneu" data={canMap.get("Temperatura pneu")}/>
      <LineCanChart title="RPM"/>
      <LineCanChart title=""/>
      <LineCanChart title=""/>
      <LineCanChart title=""/>
      <LineCanChart title=""/>
    </div>
  );
}
