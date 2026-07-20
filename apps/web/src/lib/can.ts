"use client"

import { Dispatch, SetStateAction } from "react";

export function canParser(message : Uint8Array, setCanList: Dispatch<SetStateAction<Map<string, [{timestamp : number, data : number}]>>>){
  const timestamp = uint8ArrayToNumber(message, 15, 8, true);
  const canId     = uint8ArrayToNumber(message, 23, 8, true);
  const canData   = uint8ArrayToNumber(message, 28, 8, true);
  console.log(timestamp);
  console.log(canId);
  console.log(canData);
  switch(Number(canId)){
    case 0x00000000:
      break;
    case 0x00000001:
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
