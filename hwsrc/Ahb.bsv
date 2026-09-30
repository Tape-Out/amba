package Ahb;

import RegIf::*;

// AHB5 发起方那一侧的信号（AMBA 5 AHB，IHI 0033C）。从我们这边看，别人的核露出来的
// AHB 发起口就是这个形状：它驱的是值方法，它收的是 Action 方法，名字照规范。
interface AhbMgrPins;
  (* always_ready *) method Bit#(32) haddr;
  (* always_ready *) method Bit#(1)  hwrite;
  (* always_ready *) method Bit#(2)  htrans;
  (* always_ready *) method Bit#(3)  hsize;
  (* always_ready *) method Bit#(3)  hburst;
  (* always_ready *) method Bit#(4)  hprot;
  (* always_ready *) method Bit#(1)  hmastlock;
  (* always_ready *) method Bit#(8)  hmaster;
  (* always_ready *) method Bit#(1)  hexcl;
  (* always_ready *) method Bit#(32) hwdata;
  (* always_ready, always_enabled *) method Action hready(Bit#(1) v);
  (* always_ready, always_enabled *) method Action hresp(Bit#(1) v);
  (* always_ready, always_enabled *) method Action hexokay(Bit#(1) v);
  (* always_ready, always_enabled *) method Action hrdata(Bit#(32) v);
endinterface

typedef enum { Idle, Wait, Err2 } AhbSt deriving (Bits, Eq, FShow);

// AHB 发起口接到我们的片上总线上，成为一个 RegManager。
//
// 地址相收下，数据相发请求；答复回来的那一拍放开 hready，同一拍收下一笔的地址相，
// 所以连续的传输照样流水。回错按规范两拍：先 hresp=1、hready=0，再两者都为 1（3.2.3）。
// 只有这一个发起方能碰这段地址，独占访问一律成功（hexokay=1）；多核时要一个真的
// 独占监视器，那是装配的事。
module mkAhbMgr#(AhbMgrPins p)(RegManager#(32, 32));
  Reg#(AhbSt)    st <- mkReg(Idle);
  Reg#(Bit#(32)) a  <- mkReg(0);
  Reg#(Bool)     w  <- mkReg(False);
  Reg#(Bit#(3))  sz <- mkReg(0);
  Reg#(Bool)     ex <- mkReg(False);

  Wire#(Bool)        rspV <- mkBypassWire;
  Wire#(RegRsp#(32)) rspX <- mkBypassWire;
  Wire#(Bool)        rdy  <- mkBypassWire;

  function Bit#(4) strb(Bit#(3) s, Bit#(2) lo);
    case (s)
      0: return 4'b0001 << lo;
      1: return 4'b0011 << {lo[1], 1'b0};
      default: return 4'b1111;
    endcase
  endfunction

  rule step;
    Bool done = st == Wait && rspV && !rspX.err;
    Bool bad = st == Wait && rspV && rspX.err;
    // 这一拍 hready 为高，发起方的地址相就被收下
    Bool take = st == Idle || done || st == Err2;
    p.hready((take && !bad) ? 1 : 0);
    p.hresp((bad || st == Err2) ? 1 : 0);
    p.hexokay((done && ex) ? 1 : 0);
    p.hrdata(done ? rspX.rdata : 0);
    if (bad) st <= Err2;
    else if (take) begin
      if (p.htrans[1] == 1) begin
        a <= p.haddr;
        w <= p.hwrite == 1;
        sz <= p.hsize;
        ex <= p.hexcl == 1;
        st <= Wait;
      end else st <= Idle;
    end
  endrule

  method Bool valid = st == Wait;
  method RegReq#(32, 32) req = RegReq { addr: {a[31:2], 2'b00}, write: w,
                                        wdata: w ? p.hwdata : 0,
                                        wstrb: w ? strb(sz, a[1:0]) : 0 };
  method Action ready(Bool x);
    rdy <= x;
  endmethod
  method Action resp(Bool v, RegRsp#(32) x);
    rspV <= v;
    rspX <= x;
  endmethod
endmodule

endpackage
