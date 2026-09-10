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
  // PRDATA 与 PSLVERR 必须在 PREADY 抬着的**那一拍**就位（IHI 0024D 附录 A：
  // 「must be valid when PSEL, PENABLE, and PREADY are asserted」；3.3.1 同旨，
  // 「The Completer must provide the data before the end of the read transfer」）。
  // 寄存器要下一拍才出得来——PREADY 恒高时传输就在 ACCESS 那一拍结束，主机采到的
  // 是上一笔读的数据。所以走线不走寄存器：从 PADDR 到 PRDATA 是一条组合通路，
  // 零等待从设备本来就是这个样子。
  Wire#(Bit#(dw)) rdataW  <- mkDWire(0);
  Wire#(Bool)     slverrW <- mkDWire(False);

  method Action req(paddr, pprot, psel, penable, pwrite, pwdata, pstrb);
    if (psel && penable) begin
      let rsp <- rf.access(RegReq { addr: paddr, write: pwrite,
                                    wdata: pwdata, wstrb: pstrb });
      rdataW  <= rsp.rdata;
      slverrW <= rsp.err;
    end
  endmethod

  method Bool     pready  = True;   // 零等待寄存器从设备
  method Bit#(dw) prdata  = rdataW;
  method Bool     pslverr = slverrW;
endmodule


// 会停顿的目标接 APB4：一层纯转接，没有自己的状态。
//
// 目标没收下或还没答，`rspValid` 就是假，PREADY 于是压着，主机按 3.1.2 与 3.3.2
// 顶住 PADDR/PWRITE/PWDATA/PSTRB 不动——恰好就是 `RegTarget` 要求发起方做的事，
// 两边的约定是同一条，所以中间不需要缓冲。
//
// 前提：这个目标只有这一个发起方。`rspValid` 不带标签，谁的答复都长一个样，
// 片上还有别的发起方时要走 mkApb4Manager 去排队，不能用这个。
module mkApb4BindT#(RegTarget#(aw, dw) rf)(Apb4SlavePins#(aw, dw));
  method Action req(paddr, pprot, psel, penable, pwrite, pwdata, pstrb);
    rf.req(psel && penable, RegReq { addr: paddr, write: pwrite,
                                     wdata: pwdata, wstrb: pstrb });
  endmethod

  method Bool     pready  = rf.rspValid;
  method Bit#(dw) prdata  = rf.rsp.rdata;
  method Bool     pslverr = rf.rsp.err;
endmodule


// 反方向：把已有 APB4 引脚的第三方从设备收编进中立契约，之后它进地址图，与自研
// IP 无差别。形态同 Chisel 的 BlackBox，但 BVI 薄壳由 fusesoc fork 按协议模板生成，
// 不必手写。
//
// 收编出来的是**会停顿的目标**而不是 RegIf：APB4 最少要 SETUP 与 ACCESS 两拍，而
// RegIf 的 access 是一次就答的动作值，两者对不上。原来那一版硬凑成 RegIf，发出请求
// 的同一拍就把 PRDATA 读回来当答复——读到的是上一笔的数据。
//
// 前提是对方的 PREADY 与 PRDATA 不是从 PADDR 组合出来的。第三方是 BVI 黑盒，调度由
// BVI 声明，不会撞上；我们自己的零等待绑定器 mkApb4Bind 却是组合直通，两个直接对接
// 会成为同一条线又写又读（G0004），中间要隔一级寄存器。
module mkApb4Adopt#(Apb4SlavePins#(aw, dw) sl)(RegTarget#(aw, dw));
  // 状态只有 step 一条规则写，方法只发线
  Reg#(Bit#(2))          st   <- mkReg(0);   // 0=IDLE 1=SETUP 2=ACCESS
  Reg#(RegReq#(aw, dw))  held <- mkReg(unpack(0));
  Reg#(Bool)             ansV <- mkReg(False);
  Reg#(RegRsp#(dw))      ansX <- mkReg(unpack(0));

  Wire#(Bool)            takeV <- mkDWire(False);
  Wire#(RegReq#(aw, dw)) takeR <- mkDWire(unpack(0));

  rule step;
    Bool psel = st != 0;
    Bool pen  = st == 2;
    // 读传输里 PSTRB 必须全低（3.2）。发起方一律填满是常见写法，而认识 APB4 的
    // 只有这一处，所以在这里挡掉，不去改每一个发起方。
    sl.req(held.addr, 3'b000, psel, pen, held.write, held.wdata,
           held.write ? held.wstrb : 0);

    Bit#(2)         nst = st;
    Bool            nan = False;
    RegRsp#(dw)     nax = ansX;
    RegReq#(aw, dw) nhd = held;

    if (st == 0) begin
      if (takeV) begin
        nhd = takeR;
        nst = 1;
      end
    end else if (st == 1)
      nst = 2;
    else if (sl.pready) begin
      nst = 0;
      nan = True;
      nax = RegRsp { rdata: sl.prdata, err: sl.pslverr };
    end

    st   <= nst;
    held <= nhd;
    ansV <= nan;
    ansX <= nax;
  endrule

  method Action req(Bool valid, RegReq#(aw, dw) r);
    if (valid && st == 0 && !ansV) begin
      takeV <= True;
      takeR <= r;
    end
  endmethod
  // 答复的那一拍不再收新的：发起方要到下一拍才会撤掉 valid，不挡就会重发一遍
  method Bool ready = st == 0 && !ansV;
  method Bool rspValid = ansV;
  method RegRsp#(dw) rsp = ansX;
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
