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

endpackage
