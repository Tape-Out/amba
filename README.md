# amba

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-Apache--2.0-blue)

AMBA protocol suite in Bluespec: pin-level interfaces and binders that attach a
bus-neutral `RegIf` or a stalling `RegTarget` (from `hwcore`) to real bus pins.

## Status

| Protocol | Spec | Pins | Binders |
| :--: | :--: | :--: | :--: |
| APB4 | ARM IHI 0024 | `Apb4SlavePins` | `mkApb4Bind`, `mkApb4BindT`, `mkApb4Adopt`, `mkApb4Manager` |
| AXI4-Lite | ARM IHI 0022 | `Axi4LiteSlavePins`, `Axi4LiteMasterPins` | `mkAxi4LiteBind`, `mkAxi4LiteBindT`, `mkAxi4LiteAdopt`, `mkAxi4LiteWire` |
| AXI4-Stream | ARM IHI 0051 | planned | planned |
| AXI4 | ARM IHI 0022 | planned | planned |
| AHB-Lite | ARM IHI 0033 | planned | planned |

The AXI4-Lite binders register every output, so there is no combinational path from an input to an output. A write response waits for both the write address and the write data handshake, in either order, and a response holds until BREADY or RREADY takes it. Target errors answer SLVERR. One transaction per direction is in flight at a time. When a read and a write are both ready, they take turns.

`mkAxi4LiteAdopt` is the requester side: it turns AXI4-Lite completer pins, such as a third-party slave, into a stalling `RegTarget`. It sends one transaction at a time, raises AWVALID and WVALID together, holds every VALID with its address and data until the handshake, and treats SLVERR, DECERR and EXOKAY as errors. `mkAxi4LiteWire` joins completer pins driven inside the chip to requester pins that leave it. Selecting AXI4-Lite as the bus of an assembly is not implemented yet.

## Why binders instead of per-IP wrappers

An IP implements `RegIf` and nothing else. Attaching it to a bus is one line at
integration time:

```bsv
RegIf#(32, 32)          bus <- mkFabric(devs);
Apb4SlavePins#(32, 32)  apb <- mkApb4Bind(bus);
```

Changing the bus means changing that one line. Ten IPs across four buses would
otherwise need forty wrappers; here it is ten IPs plus four binders.

## Bridges

Protocol bridges are needed in two cases: attaching third-party IP that already
exposes bus pins, and building a bus hierarchy where a fast fabric carries a
slower subordinate segment. Our own IPs need no bridge, since the bus is chosen
at bind time.

APB4 and AXI4-Lite are instances of the `Bus` type class in `hwcore`, whose
`bindT` and `adopt` are the binders above. A bridge from one bus to another is
then the generic `bridge` function; synthesizable bridges with requester pins
live in the [`bridge`](https://github.com/Tape-Out/bridge) library.

## Specification sources

The specifications this IP is implemented against, with their links, digests and the clause-by-clause comparison, are kept on the [`spec` branch](https://github.com/Tape-Out/amba/tree/spec).

## License

Apache License 2.0.
