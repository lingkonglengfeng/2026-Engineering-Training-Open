/* Browser/Node shared protocol and serial lifecycle. */
(function(root){
  'use strict';
  const encoder=new TextEncoder();
  function hex(bytes){return Array.from(bytes,b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ');}
  function textPacket(value,ending='\r\n'){
    value=value.trim();
    if(!value || /[\r\n\x00-\x1f\x7f]/.test(value))throw Error('一次发送一条指令；不允许控制字符。');
    const limit=value.startsWith('&')?31:63;
    if(encoder.encode(value).length>limit)throw Error(`指令超过固件 ${limit} 字节上限，请缩短参数。`);
    if(!/^[\x20-\x7e]+$/.test(value))throw Error('固件指令只接受ASCII字符。');
    return encoder.encode(value+ending);
  }
  function parseHex(value){
    const parts=value.trim().split(/\s+/);
    if(!value.trim() || parts.some(p=>! /^(?:0x)?[0-9a-f]{2}$/i.test(p)))throw Error('HEX每字节两位，用空格分隔，例如 6B 80 80 00 00 00 00 27。');
    if(parts.length>256)throw Error('每次最多256字节。');
    return Uint8Array.from(parts,p=>parseInt(p.replace(/^0x/i,''),16));
  }
  function crc8(data){let crc=0;for(const b of data){crc^=b;for(let i=0;i<8;i++)crc=crc&128?((crc<<1)^7)&255:(crc<<1)&255;}return crc;}
  function makeSet(mode,station,target,hz,seq){
    if(![mode,station,target,hz,seq].every(Number.isInteger)||seq<0||seq>15)throw Error('SET参数必须为整数，SEQ为0..15。');
    const okay=mode===0?station===0&&target===0&&hz===0:
      mode===2?station===1&&target>=0&&target<=6&&hz>=1&&hz<=50:
      mode===3?[2,3].includes(station)&&target===2&&hz>=1&&hz<=50:
      mode===8?station===1&&target===0&&hz>=1&&hz<=50:
      mode===6?station===4&&target===0&&hz>=1&&hz<=50:false;
    if(!okay)throw Error('模式/工位/ID/Hz不符合相机SET规则。请先选择预设模式。');
    const data=[0x6b,0x80,0x80|(seq<<3),mode,station,target,hz];return Uint8Array.from([...data,crc8(data)]);
  }
  class LineReader{
    constructor(){this.decoder=new TextDecoder();this.buffer='';}
    feed(bytes,final=false){this.buffer+=this.decoder.decode(bytes,{stream:!final});const lines=this.buffer.split(/\r\n|\r|\n/);this.buffer=lines.pop();const ready=lines.filter(Boolean);if(this.buffer.length>4096){ready.push(this.buffer.slice(0,4096)+' [长行截断]');this.buffer=this.buffer.slice(4096);}if(final&&this.buffer){ready.push(this.buffer);this.buffer='';}return ready;}
  }
  class FrameReader{
    constructor(){this.buffer=[];this.last=-Infinity;this.errors=0;}
    feed(bytes,now){if(now-this.last>50)this.buffer=[];this.last=now;this.buffer.push(...bytes);const out=[];
      while(this.buffer.length){if(this.buffer[0]!==0x6b){this.buffer.shift();continue;}if(this.buffer.length<8)break;
        const frame=this.buffer.slice(0,8);if(crc8(frame.slice(0,7))!==frame[7]){this.errors++;this.buffer.shift();continue;}out.push(frame);this.buffer.splice(0,8);}
      return out;
    }
  }
  function describe(frame){const type=frame[1],meta=frame[2],id=meta&7,seq=(meta>>3)&15,valid=!!(meta&128),d=frame.slice(3,7);let content='D='+hex(d);
    if([2,3,9].includes(type)){const signed=(a,b)=>{const n=a|(b<<8);return(n&32768?n-65536:n)/10;};content=`X=${signed(d[0],d[1])} Y=${signed(d[2],d[3])} px（右/下正）`;}
    if(type===0x83)content=`数量=${d[0]} 工位=${d[1]} SET_SEQ=${d[2]}`;
    if(type===0x85)content=`盘状态=${['UNKNOWN','SEARCH','ACQUIRE','FOUND'][d[0]]||d[0]} 工位=${d[1]} SET_SEQ=${d[2]}`;
    if(type===0x81)content=`ACK 模式=${d[0]} 工位=${d[1]} ID=${d[2]} 结果=${d[3]}`;
    if(type===7)content=`QR组${id}: ${d.slice(0,3).join('')} 共${d[3]}组 SET_SEQ=${seq}`;
    return `TYPE=${type.toString(16).padStart(2,'0')} SEQ=${seq} ID=${id} VALID=${+valid} ${content}`;
  }
  class SerialLink{
    constructor(onData=()=>{},onState=()=>{}){this.onData=onData;this.onState=onState;this.port=null;this.writer=null;this.reader=null;this.queue=[];this.pumping=false;this.connected=false;this.closing=null;}
    async open(port,options){if(this.port)throw Error('串口已打开或正在关闭。');await port.open(options);this.port=port;this.writer=port.writable.getWriter();this.connected=true;this.onState(true);this.readTask=this.readLoop();}
    async readLoop(){try{while(this.connected&&this.port.readable){const reader=this.port.readable.getReader();this.reader=reader;try{while(this.connected){const {value,done}=await reader.read();if(done)break;if(value)this.onData(value);}}finally{reader.releaseLock();if(this.reader===reader)this.reader=null;}}}catch(e){if(this.connected)this.onState(false,e);}finally{if(this.connected)void this.close().catch(e=>this.onState(false,e));}}
    send(bytes,priority=false){if(!this.connected)return Promise.reject(Error('请先连接串口或进入演示模式。'));if(priority)this.clearQueue();if(this.queue.length>=64)return Promise.reject(Error('发送队列已满。'));
      return new Promise((resolve,reject)=>{const item={bytes:new Uint8Array(bytes),resolve,reject};if(priority)this.queue.unshift(item);else this.queue.push(item);void this.pump();});}
    clearQueue(){for(const item of this.queue.splice(0))item.reject(Error('待发指令已取消。'));}
    async pump(){if(this.pumping)return;this.pumping=true;try{while(this.connected&&this.queue.length){const item=this.queue.shift();try{await this.writer.write(item.bytes);item.resolve();}catch(e){item.reject(e);this.connected=false;this.clearQueue();this.onState(false,e);void this.close().catch(e=>this.onState(false,e));break;}}}finally{this.pumping=false;}}
    close(){if(this.closing)return this.closing;this.connected=false;this.clearQueue();this.onState(false);this.closing=this.finishClose();return this.closing;}
    async finishClose(){try{if(this.reader)await this.reader.cancel();if(this.writer){await this.writer.abort().catch(()=>{});this.writer.releaseLock();this.writer=null;}if(this.port)await this.port.close();}finally{this.port=null;this.closing=null;}}
  }
  const api={hex,textPacket,parseHex,crc8,makeSet,LineReader,FrameReader,describe,SerialLink};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.SerialCore=api;
})(globalThis);
