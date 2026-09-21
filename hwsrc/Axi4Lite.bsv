package Axi4Lite;

import RegIf::*;
import Bus::*;

// AMBA AXI4-Lite（ARM IHI 0022H 第 B1 章）：五个通道各自 VALID/READY 握手（A3.2.1），突发长度恒为 1，
// 没有 ID（B1.1.4），所以同一方向一次只让一笔在途、答复按到达次序出。
// 输出一律出自寄存器：A3.2.1 要求接口的输入与输出之间不能有组合通路，READY 也就不看同拍的 VALID。

interface Axi4LiteSlavePins#(numeric type aw, numeric type dw);
  (* always_ready, always_enabled, prefix = "" *)
  method Action aw_in((* port = "awvalid" *) Bool     awvalid,
                      (* port = "awaddr" *)  Bit#(aw) awaddr,
                      (* port = "awprot" *)  Bit#(3)  awprot);
  (* always_ready, result = "awready" *) method Bool awready;

  (* always_ready, always_enabled, prefix = "" *)
  method Action w_in((* port = "wvalid" *) Bool               wvalid,
                     (* port = "wdata" *)  Bit#(dw)           wdata,
                     (* port = "wstrb" *)  Bit#(TDiv#(dw, 8)) wstrb);
  (* always_ready, result = "wready" *) method Bool wready;

  (* always_ready, result = "bvalid" *) method Bool    bvalid;
  (* always_ready, result = "bresp" *)  method Bit#(2) bresp;
  (* always_ready, always_enabled, prefix = "" *)
  method Action b_in((* port = "bready" *) Bool bready);

  (* always_ready, always_enabled, prefix = "" *)
  method Action ar_in((* port = "arvalid" *) Bool     arvalid,
                      (* port = "araddr" *)  Bit#(aw) araddr,
                      (* port = "arprot" *)  Bit#(3)  arprot);
  (* always_ready, result = "arready" *) method Bool arready;

  (* always_ready, result = "rvalid" *) method Bool     rvalid;
  (* always_ready, result = "rdata" *)  method Bit#(dw) rdata;
  (* always_ready, result = "rresp" *)  method Bit#(2)  rresp;
  (* always_ready, always_enabled, prefix = "" *)
  method Action r_in((* port = "rready" *) Bool rready);
endinterface

// 响应编码（A3.4.4）：AXI4-Lite 不用 EXOKAY（B1.1.1）；我们的错误都是完成方自己报的，用 SLVERR
function Bit#(2) respOf(Bool err) = err ? 2'b10 : 2'b00;

// 零等待目标接 AXI4-Lite
module mkAxi4LiteBind#(RegIf#(aw, dw) rf)(Axi4LiteSlavePins#(aw, dw));
  Wire#(Tuple2#(Bool, Bit#(aw)))                     awIn <- mkBypassWire;
  Wire#(Tuple3#(Bool, Bit#(dw), Bit#(TDiv#(dw, 8)))) wIn  <- mkBypassWire;
  Wire#(Bool)                                        bIn  <- mkBypassWire;
  Wire#(Tuple2#(Bool, Bit#(aw)))                     arIn <- mkBypassWire;
  Wire#(Bool)                                        rIn  <- mkBypassWire;

  Reg#(Maybe#(Bit#(aw)))                              awH <- mkReg(tagged Invalid);
  Reg#(Maybe#(Tuple2#(Bit#(dw), Bit#(TDiv#(dw, 8))))) wH  <- mkReg(tagged Invalid);
  Reg#(Maybe#(Bit#(aw)))                              arH <- mkReg(tagged Invalid);
  Reg#(Bool)     bV     <- mkReg(False);
  Reg#(Bit#(2))  bR     <- mkReg(0);
  Reg#(Bool)     rV     <- mkReg(False);
  Reg#(Bit#(dw)) rD     <- mkReg(0);
  Reg#(Bit#(2))  rR     <- mkReg(0);
  Reg#(Bool)     favorR <- mkReg(False);

  // 答复没被取走之前不收同一方向的下一笔：没有 ID，答复只能按次序出
  Bool awRdy = !isValid(awH) && !bV;
  Bool wRdy  = !isValid(wH) && !bV;
  Bool arRdy = !isValid(arH) && !rV;

  rule step;
    match {.awv, .awa} = awIn;
    match {.wv, .wd, .ws} = wIn;
    match {.arv, .ara} = arIn;
    let nAw = (awv && awRdy) ? tagged Valid awa : awH;
    let nW  = (wv && wRdy) ? tagged Valid tuple2(wd, ws) : wH;
    let nAr = (arv && arRdy) ? tagged Valid ara : arH;
    Bool     nbV = bV && !bIn;
    Bool     nrV = rV && !rIn;
    Bit#(2)  nbR = bR;
    Bit#(2)  nrR = rR;
    Bit#(dw) nrD = rD;

    // 写响应要等地址与数据两次握手都完成（A3.3.1，AXI4 的写事务依赖）
    Bool canW = isValid(nAw) && isValid(nW) && !bV;
    Bool canR = isValid(nAr) && !rV;
    // 读写同拍就绪时轮着来，免得一路一直占着
    Bool doW = canW && (!canR || !favorR);
    Bool doR = canR && !doW;
    if (doW) begin
      match {.d, .s} = fromMaybe(?, nW);
      let x <- rf.access(RegReq { addr: fromMaybe(?, nAw), write: True, wdata: d, wstrb: s });
      nbV = True; nbR = respOf(x.err);
      nAw = tagged Invalid; nW = tagged Invalid;
    end else if (doR) begin
      let x <- rf.access(RegReq { addr: fromMaybe(?, nAr), write: False, wdata: 0, wstrb: 0 });
      nrV = True; nrD = x.rdata; nrR = respOf(x.err);
      nAr = tagged Invalid;
    end
    if (doW) favorR <= True;
    else if (doR) favorR <= False;

    awH <= nAw; wH <= nW; arH <= nAr;
    bV <= nbV; bR <= nbR;
    rV <= nrV; rD <= nrD; rR <= nrR;
  endrule

  method Action aw_in(Bool awvalid, Bit#(aw) awaddr, Bit#(3) awprot);
    awIn <= tuple2(awvalid, awaddr);
  endmethod
  method Bool awready = awRdy;
  method Action w_in(Bool wvalid, Bit#(dw) wdata, Bit#(TDiv#(dw, 8)) wstrb);
    wIn <= tuple3(wvalid, wdata, wstrb);
  endmethod
  method Bool wready = wRdy;
  method Bool bvalid = bV;
  method Bit#(2) bresp = bR;
  method Action b_in(Bool bready);
    bIn <= bready;
  endmethod
  method Action ar_in(Bool arvalid, Bit#(aw) araddr, Bit#(3) arprot);
    arIn <= tuple2(arvalid, araddr);
  endmethod
  method Bool arready = arRdy;
  method Bool rvalid = rV;
  method Bit#(dw) rdata = rD;
  method Bit#(2) rresp = rR;
  method Action r_in(Bool rready);
    rIn <= rready;
  endmethod
endmodule


// 会停顿的目标接 AXI4-Lite。握手的收法与上面相同，只是凑齐的一笔交给目标、等 rspValid 再出答复。
module mkAxi4LiteBindT#(RegTarget#(aw, dw) t)(Axi4LiteSlavePins#(aw, dw));
  Wire#(Tuple2#(Bool, Bit#(aw)))                     awIn <- mkBypassWire;
  Wire#(Tuple3#(Bool, Bit#(dw), Bit#(TDiv#(dw, 8)))) wIn  <- mkBypassWire;
  Wire#(Bool)                                        bIn  <- mkBypassWire;
  Wire#(Tuple2#(Bool, Bit#(aw)))                     arIn <- mkBypassWire;
  Wire#(Bool)                                        rIn  <- mkBypassWire;

  Reg#(Maybe#(Bit#(aw)))                              awH  <- mkReg(tagged Invalid);
  Reg#(Maybe#(Tuple2#(Bit#(dw), Bit#(TDiv#(dw, 8))))) wH   <- mkReg(tagged Invalid);
  Reg#(Maybe#(Bit#(aw)))                              arH  <- mkReg(tagged Invalid);
  Reg#(Maybe#(RegReq#(aw, dw)))                       pend <- mkReg(tagged Invalid);
  Reg#(Bool)     bV     <- mkReg(False);
  Reg#(Bit#(2))  bR     <- mkReg(0);
  Reg#(Bool)     rV     <- mkReg(False);
  Reg#(Bit#(dw)) rD     <- mkReg(0);
  Reg#(Bit#(2))  rR     <- mkReg(0);
  Reg#(Bool)     favorR <- mkReg(False);

  Bool awRdy = !isValid(awH) && !bV;
  Bool wRdy  = !isValid(wH) && !bV;
  Bool arRdy = !isValid(arH) && !rV;

  // 发与收分两条规则，道理同 mkPipe：地址图那类目标的 rspValid 是从 req 那条线组合出来的，
  // 同一条规则里又写又读就成环。两条规则仍在同一拍跑
  rule send;
    t.req(isValid(pend), fromMaybe(unpack(0), pend));
  endrule

  rule step;
    match {.awv, .awa} = awIn;
    match {.wv, .wd, .ws} = wIn;
    match {.arv, .ara} = arIn;
    let nAw = (awv && awRdy) ? tagged Valid awa : awH;
    let nW  = (wv && wRdy) ? tagged Valid tuple2(wd, ws) : wH;
    let nAr = (arv && arRdy) ? tagged Valid ara : arH;
    Bool     nbV = bV && !bIn;
    Bool     nrV = rV && !rIn;
    Bit#(2)  nbR = bR;
    Bit#(2)  nrR = rR;
    Bit#(dw) nrD = rD;
    Maybe#(RegReq#(aw, dw)) np = pend;

    if (pend matches tagged Valid .q &&& t.rspValid) begin
      let x = t.rsp;
      if (q.write) begin nbV = True; nbR = respOf(x.err); end
      else begin nrV = True; nrD = x.rdata; nrR = respOf(x.err); end
      np = tagged Invalid;
    end else if (!isValid(pend)) begin
      Bool goW = isValid(nAw) && isValid(nW) && !bV;
      Bool goR = isValid(nAr) && !rV;
      Bool sendW = goW && (!goR || !favorR);
      Bool sendR = goR && !sendW;
      if (sendW) begin
        match {.d, .s} = fromMaybe(?, nW);
        np = tagged Valid RegReq { addr: fromMaybe(?, nAw), write: True, wdata: d, wstrb: s };
        nAw = tagged Invalid; nW = tagged Invalid;
        favorR <= True;
      end else if (sendR) begin
        np = tagged Valid RegReq { addr: fromMaybe(?, nAr), write: False, wdata: 0, wstrb: 0 };
        nAr = tagged Invalid;
        favorR <= False;
      end
    end

    awH <= nAw; wH <= nW; arH <= nAr; pend <= np;
    bV <= nbV; bR <= nbR;
    rV <= nrV; rD <= nrD; rR <= nrR;
  endrule

  method Action aw_in(Bool awvalid, Bit#(aw) awaddr, Bit#(3) awprot);
    awIn <= tuple2(awvalid, awaddr);
  endmethod
  method Bool awready = awRdy;
  method Action w_in(Bool wvalid, Bit#(dw) wdata, Bit#(TDiv#(dw, 8)) wstrb);
    wIn <= tuple3(wvalid, wdata, wstrb);
  endmethod
  method Bool wready = wRdy;
  method Bool bvalid = bV;
  method Bit#(2) bresp = bR;
  method Action b_in(Bool bready);
    bIn <= bready;
  endmethod
  method Action ar_in(Bool arvalid, Bit#(aw) araddr, Bit#(3) arprot);
    arIn <= tuple2(arvalid, araddr);
  endmethod
  method Bool arready = arRdy;
  method Bool rvalid = rV;
  method Bit#(dw) rdata = rD;
  method Bit#(2) rresp = rR;
  method Action r_in(Bool rready);
    rIn <= rready;
  endmethod
endmodule

// ---------------- 发起方一侧：收编 AXI4-Lite 完成方、出芯片的引脚、总线类型类的实例 ----------------

// 收编 AXI4-Lite 完成方，出来的是会停顿的目标。一次一笔：写等 AW、W 两次握手再等 B，读等 AR 再等 R。
// VALID、地址、数据都出自寄存器：A3.2.1 不许等 READY 再抬 VALID，握手之前不许撤；AW 与 W 同一拍一起抬（A3.3.1）
module mkAxi4LiteAdopt#(Axi4LiteSlavePins#(aw, dw) sl)(RegTarget#(aw, dw));
  Reg#(UInt#(3))         st    <- mkReg(0);   // 0 空闲 · 1 写，等 AW 与 W · 2 写，等 B · 3 读，等 AR · 4 读，等 R
  Reg#(Bool)             awOk  <- mkReg(False);
  Reg#(Bool)             wOk   <- mkReg(False);
  Reg#(RegReq#(aw, dw))  held  <- mkReg(unpack(0));
  Reg#(Bool)             ansV  <- mkReg(False);
  Reg#(RegRsp#(dw))      ansX  <- mkReg(unpack(0));
  Wire#(Bool)            takeV <- mkDWire(False);
  Wire#(RegReq#(aw, dw)) takeR <- mkDWire(unpack(0));

  Bool awv = st == 1 && !awOk;
  Bool wv  = st == 1 && !wOk;

  // 发与收分两条规则：完成方可以等 VALID 再抬 READY（A3.2.1），READY 就可能是从这边的输出组合出来的，
  // 同一条规则里又写又读会成环，道理同 mkPipe
  rule send;
    sl.aw_in(awv, held.addr, 3'b000);
    sl.w_in(wv, held.wdata, held.wstrb);
    sl.b_in(st == 2);
    sl.ar_in(st == 3, held.addr, 3'b000);
    sl.r_in(st == 4);
  endrule

  rule take;
    UInt#(3)        nst = st;
    Bool            nAw = awOk;
    Bool            nW  = wOk;
    Bool            nan = False;
    RegRsp#(dw)     nax = ansX;
    RegReq#(aw, dw) nhd = held;
    case (st)
      0: if (takeV) begin nhd = takeR; nst = takeR.write ? 1 : 3; nAw = False; nW = False; end
      1: begin
           if (awv && sl.awready) nAw = True;
           if (wv && sl.wready) nW = True;
           if (nAw && nW) nst = 2;
         end
      // EXOKAY 只回独占访问，这边从不发独占访问，收到也当错（A3.4.4、B1.1.1）
      2: if (sl.bvalid) begin nst = 0; nan = True; nax = RegRsp { rdata: 0, err: sl.bresp != 2'b00 }; end
      3: if (sl.arready) nst = 4;
      4: if (sl.rvalid) begin nst = 0; nan = True; nax = RegRsp { rdata: sl.rdata, err: sl.rresp != 2'b00 }; end
      default: noAction;
    endcase
    st <= nst; awOk <= nAw; wOk <= nW; held <= nhd; ansV <= nan; ansX <= nax;
  endrule

  method Action req(Bool valid, RegReq#(aw, dw) r);
    if (valid && st == 0 && !ansV) begin takeV <= True; takeR <= r; end
  endmethod
  // 答复的那一拍不再收新的：发起方要到下一拍才撤 valid
  method Bool ready = st == 0 && !ansV;
  method Bool rspValid = ansV;
  method RegRsp#(dw) rsp = ansX;
endmodule

// 发起方引脚：完成方引脚的对偶，片外接一个 AXI4-Lite 完成方
interface Axi4LiteMasterPins#(numeric type aw, numeric type dw);
  (* always_ready, result = "awvalid" *) method Bool               awvalid;
  (* always_ready, result = "awaddr" *)  method Bit#(aw)           awaddr;
  (* always_ready, result = "awprot" *)  method Bit#(3)            awprot;
  (* always_ready, always_enabled, prefix = "" *)
  method Action aw_ready((* port = "awready" *) Bool r);

  (* always_ready, result = "wvalid" *)  method Bool               wvalid;
  (* always_ready, result = "wdata" *)   method Bit#(dw)           wdata;
  (* always_ready, result = "wstrb" *)   method Bit#(TDiv#(dw, 8)) wstrb;
  (* always_ready, always_enabled, prefix = "" *)
  method Action w_ready((* port = "wready" *) Bool r);

  (* always_ready, always_enabled, prefix = "" *)
  method Action b_rsp((* port = "bvalid" *) Bool v, (* port = "bresp" *) Bit#(2) resp);
  (* always_ready, result = "bready" *)  method Bool               bready;

  (* always_ready, result = "arvalid" *) method Bool               arvalid;
  (* always_ready, result = "araddr" *)  method Bit#(aw)           araddr;
  (* always_ready, result = "arprot" *)  method Bit#(3)            arprot;
  (* always_ready, always_enabled, prefix = "" *)
  method Action ar_ready((* port = "arready" *) Bool r);

  (* always_ready, always_enabled, prefix = "" *)
  method Action r_rsp((* port = "rvalid" *) Bool v, (* port = "rdata" *) Bit#(dw) data,
                      (* port = "rresp" *) Bit#(2) resp);
  (* always_ready, result = "rready" *)  method Bool               rready;
endinterface

// 线网：一边是给片内发起方驱动的完成方引脚，一边是出芯片的发起方引脚，中间只有线
interface Axi4LiteWire#(numeric type aw, numeric type dw);
  interface Axi4LiteSlavePins#(aw, dw)  slave;
  interface Axi4LiteMasterPins#(aw, dw) master;
endinterface

module mkAxi4LiteWire(Axi4LiteWire#(aw, dw));
  Wire#(Tuple3#(Bool, Bit#(aw), Bit#(3)))            awW  <- mkBypassWire;
  Wire#(Tuple3#(Bool, Bit#(dw), Bit#(TDiv#(dw, 8)))) wW   <- mkBypassWire;
  Wire#(Bool)                                        bRdy <- mkBypassWire;
  Wire#(Tuple3#(Bool, Bit#(aw), Bit#(3)))            arW  <- mkBypassWire;
  Wire#(Bool)                                        rRdy <- mkBypassWire;
  Wire#(Bool)                                        awR  <- mkBypassWire;
  Wire#(Bool)                                        wR   <- mkBypassWire;
  Wire#(Tuple2#(Bool, Bit#(2)))                      bIn  <- mkBypassWire;
  Wire#(Bool)                                        arR  <- mkBypassWire;
  Wire#(Tuple3#(Bool, Bit#(dw), Bit#(2)))            rIn  <- mkBypassWire;

  interface Axi4LiteSlavePins slave;
    method Action aw_in(Bool v, Bit#(aw) a, Bit#(3) p); awW <= tuple3(v, a, p); endmethod
    method Bool awready = awR;
    method Action w_in(Bool v, Bit#(dw) d, Bit#(TDiv#(dw, 8)) s); wW <= tuple3(v, d, s); endmethod
    method Bool wready = wR;
    method Bool bvalid = tpl_1(bIn);
    method Bit#(2) bresp = tpl_2(bIn);
    method Action b_in(Bool r); bRdy <= r; endmethod
    method Action ar_in(Bool v, Bit#(aw) a, Bit#(3) p); arW <= tuple3(v, a, p); endmethod
    method Bool arready = arR;
    method Bool rvalid = tpl_1(rIn);
    method Bit#(dw) rdata = tpl_2(rIn);
    method Bit#(2) rresp = tpl_3(rIn);
    method Action r_in(Bool r); rRdy <= r; endmethod
  endinterface

  interface Axi4LiteMasterPins master;
    method Bool               awvalid = tpl_1(awW);
    method Bit#(aw)           awaddr  = tpl_2(awW);
    method Bit#(3)            awprot  = tpl_3(awW);
    method Action aw_ready(Bool r); awR <= r; endmethod
    method Bool               wvalid  = tpl_1(wW);
    method Bit#(dw)           wdata   = tpl_2(wW);
    method Bit#(TDiv#(dw, 8)) wstrb   = tpl_3(wW);
    method Action w_ready(Bool r); wR <= r; endmethod
    method Action b_rsp(Bool v, Bit#(2) resp); bIn <= tuple2(v, resp); endmethod
    method Bool               bready  = bRdy;
    method Bool               arvalid = tpl_1(arW);
    method Bit#(aw)           araddr  = tpl_2(arW);
    method Bit#(3)            arprot  = tpl_3(arW);
    method Action ar_ready(Bool r); arR <= r; endmethod
    method Action r_rsp(Bool v, Bit#(dw) data, Bit#(2) resp); rIn <= tuple3(v, data, resp); endmethod
    method Bool               rready  = rRdy;
  endinterface
endmodule

instance Bus#(Axi4LiteSlavePins#(aw, dw), aw, dw);
  function Module#(Axi4LiteSlavePins#(aw, dw)) bindT(RegTarget#(aw, dw) t) = mkAxi4LiteBindT(t);
  function Module#(RegTarget#(aw, dw)) adopt(Axi4LiteSlavePins#(aw, dw) p) = mkAxi4LiteAdopt(p);
endinstance

endpackage
