package Apb4;

import RegIf::*;

// AMBA APB4（ARM IHI 0024）：两拍时序 SETUP -> ACCESS，传输在 ACCESS 且 PREADY 时完成。
// 相对 APB3 增加 PSTRB 字节选通与 PPROT 保护属性。

interface Apb4SlavePins#(numeric type aw, numeric type dw);
  (* always_ready, always_enabled, prefix = "" *)
  method Action req(
      (* port = "paddr"   *) Bit#(aw)           paddr,
      (* port = "pprot"   *) Bit#(3)            pprot,
      (* port = "psel"    *) Bool               psel,
      (* port = "penable" *) Bool               penable,
      (* port = "pwrite"  *) Bool               pwrite,
      (* port = "pwdata"  *) Bit#(dw)           pwdata,
      (* port = "pstrb"   *) Bit#(TDiv#(dw, 8)) pstrb);
  (* always_ready, result = "pready"  *) method Bool     pready;
  (* always_ready, result = "prdata"  *) method Bit#(dw) prdata;
  (* always_ready, result = "pslverr" *) method Bool     pslverr;
endinterface

// 把中立的 RegIf 绑到 APB4 引脚。IP 侧对 APB4 一无所知。
// 传输只在 ACCESS 拍发生一次，SETUP 拍不得有副作用——旧实现漏掉这条，
// 导致一次总线写触发两次。
module mkApb4Bind#(RegIf#(aw, dw) rf)(Apb4SlavePins#(aw, dw));
  Reg#(Bit#(dw)) rdataR  <- mkReg(0);
  Reg#(Bool)     slverrR <- mkReg(False);

  method Action req(paddr, pprot, psel, penable, pwrite, pwdata, pstrb);
    if (psel && penable) begin
      let rsp <- rf.access(RegReq { addr: paddr, write: pwrite,
                                    wdata: pwdata, wstrb: pstrb });
      rdataR  <= rsp.rdata;
      slverrR <= rsp.err;
    end
  endmethod

  method Bool     pready  = True;   // 零等待寄存器从设备
  method Bit#(dw) prdata  = rdataR;
  method Bool     pslverr = slverrR;
endmodule


// 反方向：把已有 APB4 引脚的第三方从设备收编成中立 RegIf，
// 之后它进 mkFabric 与自研 IP 无差别。形态同 Chisel 的 BlackBox，
// 但 BVI 薄壳由 fusesoc fork 按协议模板生成，不必手写。
module mkApb4Adopt#(Apb4SlavePins#(aw, dw) sl)(RegIf#(aw, dw));
  // APB4 要求 SETUP 与 ACCESS 两拍。用一个状态位驱动外设走完时序，
  // 对上层仍呈现单次 access 的中立契约。
  Reg#(Bool)             phase <- mkReg(False);   // False=SETUP, True=ACCESS
  Reg#(Maybe#(RegReq#(aw, dw))) held <- mkReg(tagged Invalid);

  rule drive (held matches tagged Valid .r);
    sl.req(r.addr, 3'b000, True, phase, r.write, r.wdata, r.wstrb);
    if (phase && sl.pready) begin
      phase <= False; held <= tagged Invalid;
    end else phase <= True;
  endrule

  rule idle (held matches tagged Invalid);
    sl.req(0, 3'b000, False, False, False, 0, 0);
  endrule

  method ActionValue#(RegRsp#(dw)) access(RegReq#(aw, dw) r) if (held matches tagged Invalid);
    held <= tagged Valid r;
    return RegRsp { rdata: sl.prdata, err: sl.pslverr };
  endmethod
endmodule

// 片上有别的发起方时用这一个。零等待的 mkApb4Bind 在方法里直接调 access，
// 而那个方法是 always_enabled 的——它会把仲裁规则永久挡住，bsc 判「规则永不
// 触发」。所以这里把请求锁进寄存器，自己变成一个普通发起方去排队，
// 排到之前用 PREADY 把总线拖着。APB4 本来就允许拖。
interface Apb4Manager#(numeric type aw, numeric type dw);
  interface Apb4SlavePins#(aw, dw) pins;
  interface RegManager#(aw, dw)    mgr;
endinterface

module mkApb4Manager(Apb4Manager#(aw, dw));
  Reg#(Bool)             busy  <- mkReg(False);
  Reg#(Bool)             fin   <- mkReg(False);
  Reg#(RegReq#(aw, dw))  held  <- mkReg(unpack(0));
  Reg#(Bit#(dw))         rdataR <- mkReg(0);
  Reg#(Bool)             errR  <- mkReg(False);
  Wire#(Bool)            gnt   <- mkBypassWire;
  Wire#(Bool)            rspV  <- mkBypassWire;
  Wire#(RegRsp#(dw))     rspX  <- mkBypassWire;
  PulseWire              start <- mkPulseWire;
  Wire#(RegReq#(aw, dw)) sreq  <- mkDWire(unpack(0));

  rule launch (start && !busy && !fin);
    held <= sreq;
    busy <= True;
  endrule

  rule collect (busy && rspV);
    rdataR <= rspX.rdata;
    errR   <= rspX.err;
    busy   <= False;
    fin    <= True;
  endrule

  // PREADY 抬起来那一拍总线就走了，标记随即清掉
  rule retire (fin && !start);
    fin <= False;
  endrule

  interface Apb4SlavePins pins;
    method Action req(paddr, pprot, psel, penable, pwrite, pwdata, pstrb);
      if (psel && penable && !busy && !fin) begin
        start.send();
        sreq <= RegReq { addr: paddr, write: pwrite,
                         wdata: pwdata, wstrb: pstrb };
      end
    endmethod
    method Bool     pready  = fin;
    method Bit#(dw) prdata  = rdataR;
    method Bool     pslverr = errR;
  endinterface

  interface RegManager mgr;
    method Bool valid = busy;
    method RegReq#(aw, dw) req = held;
    method Action ready(Bool v); gnt._write(v); endmethod
    method Action resp(Bool v, RegRsp#(dw) x);
      rspV._write(v);
      rspX._write(x);
    endmethod
  endinterface
endmodule

endpackage
