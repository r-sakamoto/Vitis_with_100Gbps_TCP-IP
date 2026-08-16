//=============================================================================
// tb_toe_top.sv
//
// RTL testbench for the HLS-generated TCP Offload Engine (toe_top, 64-bit
// datapath, RX_DDR_BYPASS=1 / TCP_NODELAY=1 / WINDOW_SCALE=1 / MSS=1460).
//
// The TB provides the complete environment the TOE needs:
//   - session lookup CAM model        (m/s_axis_session_lup/upd_*)
//   - TX buffer memory model          (m_axis_txwrite/txread_* + status)
//   - RX bypass FIFO loopback         (m_axis_rxwrite_data -> s_axis_rxread_data)
//   - application BFM tasks           (listen/open/close/read/send)
//   - TCP peer model                  (builds/parses IPv4+TCP packets with
//                                      checksums on s/m_axis_tcp_data)
//
// Timing convention: all TB processes sample handshakes at posedge using
// pre-edge values (DUT registers use NBA) and drive TB->DUT signals with NBA.
//
// Test selection:  xsim ... --testplusarg TESTNAME=<name>
//   passive_open   : listen, 3WHS (peer SYN), data RX -> app, peer FIN close
//   active_open    : app open (SYN out), 3WHS, app TX data, app close (FIN out)
//   rx_out_of_order: OOO segment behavior with RX_DDR_BYPASS (drop + dup ACK)
//   fast_retransmit: 3 duplicate ACKs trigger go-back-N retransmission
//   rst_handling   : peer RST closes session; RST sent for closed port
//   zero_window    : peer advertises win=0, app send rejected, resumes on update
//=============================================================================
`timescale 1ns/1ps

module tb_toe_top;

  //---------------------------------------------------------------------------
  // Configuration
  //---------------------------------------------------------------------------
  localparam real CLK_PERIOD = 6.4;
  localparam int  MSS        = 1460;

  // IPs in "display" order (10.0.0.2 = 32'h0A000002)
  localparam bit [31:0] TOE_IP  = 32'h0A000002;   // 10.0.0.2
  localparam bit [31:0] PEER_IP = 32'h0A000001;   // 10.0.0.1

  // toe_top myIpAddress input is wire-order, byte0 = LSB
  function automatic bit [31:0] ip_to_le(input bit [31:0] ip);
    return {ip[7:0], ip[15:8], ip[23:16], ip[31:24]};
  endfunction

  typedef byte unsigned bytes_t[];

  //---------------------------------------------------------------------------
  // Clock / reset
  //---------------------------------------------------------------------------
  reg ap_clk = 0;
  reg ap_rst_n = 0;
  always #(CLK_PERIOD/2.0) ap_clk = ~ap_clk;

  //---------------------------------------------------------------------------
  // DUT signals
  //---------------------------------------------------------------------------
  // network in
  reg  [63:0] s_tcp_tdata;  reg [7:0] s_tcp_tkeep, s_tcp_tstrb; reg s_tcp_tlast;
  reg         s_tcp_tvalid; wire s_tcp_tready;
  // network out
  wire [63:0] m_tcp_tdata;  wire [7:0] m_tcp_tkeep, m_tcp_tstrb; wire m_tcp_tlast;
  wire        m_tcp_tvalid; reg  m_tcp_tready;
  // tx mem write
  wire [71:0] txwr_cmd_tdata;  wire txwr_cmd_tvalid;  reg txwr_cmd_tready;
  wire [63:0] txwr_tdata; wire [7:0] txwr_tkeep, txwr_tstrb; wire txwr_tlast;
  wire        txwr_tvalid; reg txwr_tready;
  reg  [7:0]  txwr_sts_tdata; reg txwr_sts_tvalid; wire txwr_sts_tready;
  // tx mem read
  wire [71:0] txrd_cmd_tdata;  wire txrd_cmd_tvalid;  reg txrd_cmd_tready;
  reg  [63:0] txrd_tdata; reg [7:0] txrd_tkeep, txrd_tstrb; reg txrd_tlast;
  reg         txrd_tvalid; wire txrd_tready;
  // rx buffer data path (bypass FIFO loop or DDR memory model)
  wire [63:0] rxwr_tdata; wire [7:0] rxwr_tkeep, rxwr_tstrb; wire rxwr_tlast;
  wire        rxwr_tvalid; reg rxwr_tready;
  reg  [63:0] rxrd_tdata; reg [7:0] rxrd_tkeep, rxrd_tstrb; reg rxrd_tlast;
  reg         rxrd_tvalid; wire rxrd_tready;
`ifdef NO_RX_BYPASS
  // rx buffer memory command/status interface (RX_DDR_BYPASS=0 build)
  wire [71:0] rxwr_cmd_tdata;  wire rxwr_cmd_tvalid;  reg rxwr_cmd_tready;
  wire [71:0] rxrd_cmd_tdata;  wire rxrd_cmd_tvalid;  reg rxrd_cmd_tready;
  reg  [7:0]  rxwr_sts_tdata;  reg  rxwr_sts_tvalid;  wire rxwr_sts_tready;
`else
  reg  [15:0] axis_data_count;
  reg  [15:0] axis_max_data_count = 16'd1024;
`endif
  // session lookup
  wire [95:0]  lup_req_tdata;  wire lup_req_tvalid;  reg lup_req_tready;
  reg  [119:0] lup_rsp_tdata;  reg  lup_rsp_tvalid;  wire lup_rsp_tready;
  wire [143:0] upd_req_tdata;  wire upd_req_tvalid;  reg upd_req_tready;
  reg  [151:0] upd_rsp_tdata;  reg  upd_rsp_tvalid;  wire upd_rsp_tready;
  // app interface
  reg  [15:0] listen_req_tdata; reg listen_req_tvalid; wire listen_req_tready;
  wire [7:0]  listen_rsp_tdata; wire listen_rsp_tvalid; reg listen_rsp_tready;
  wire [87:0] notif_tdata; wire notif_tvalid; reg notif_tready;
  reg  [31:0] rxreq_tdata; reg rxreq_tvalid; wire rxreq_tready;
  wire [15:0] rxrsp_meta_tdata; wire rxrsp_meta_tvalid; reg rxrsp_meta_tready;
  wire [63:0] rxrsp_tdata; wire [7:0] rxrsp_tkeep, rxrsp_tstrb; wire rxrsp_tlast;
  wire        rxrsp_tvalid; reg rxrsp_tready;
  reg  [47:0] open_req_tdata; reg open_req_tvalid; wire open_req_tready;
  wire [71:0] open_rsp_tdata; wire open_rsp_tvalid; reg open_rsp_tready;
  reg  [15:0] close_req_tdata; reg close_req_tvalid; wire close_req_tready;
  reg  [31:0] txmeta_tdata; reg txmeta_tvalid; wire txmeta_tready;
  reg  [63:0] txdata_tdata; reg [7:0] txdata_tkeep, txdata_tstrb; reg txdata_tlast;
  reg         txdata_tvalid; wire txdata_tready;
  wire [63:0] txrsp_tdata; wire txrsp_tvalid; reg txrsp_tready;
  wire [15:0] regSessionCount; wire regSessionCount_ap_vld;

  //---------------------------------------------------------------------------
  // DUT
  //---------------------------------------------------------------------------
  toe_top dut (
    .ap_clk(ap_clk), .ap_rst_n(ap_rst_n),
    .s_axis_tcp_data_TDATA(s_tcp_tdata), .s_axis_tcp_data_TKEEP(s_tcp_tkeep),
    .s_axis_tcp_data_TSTRB(s_tcp_tstrb), .s_axis_tcp_data_TLAST(s_tcp_tlast),
    .s_axis_tcp_data_TVALID(s_tcp_tvalid), .s_axis_tcp_data_TREADY(s_tcp_tready),
    .m_axis_tcp_data_TDATA(m_tcp_tdata), .m_axis_tcp_data_TKEEP(m_tcp_tkeep),
    .m_axis_tcp_data_TSTRB(m_tcp_tstrb), .m_axis_tcp_data_TLAST(m_tcp_tlast),
    .m_axis_tcp_data_TVALID(m_tcp_tvalid), .m_axis_tcp_data_TREADY(m_tcp_tready),
    .m_axis_txwrite_cmd_TDATA(txwr_cmd_tdata), .m_axis_txwrite_cmd_TVALID(txwr_cmd_tvalid), .m_axis_txwrite_cmd_TREADY(txwr_cmd_tready),
    .m_axis_txread_cmd_TDATA(txrd_cmd_tdata), .m_axis_txread_cmd_TVALID(txrd_cmd_tvalid), .m_axis_txread_cmd_TREADY(txrd_cmd_tready),
    .m_axis_txwrite_data_TDATA(txwr_tdata), .m_axis_txwrite_data_TKEEP(txwr_tkeep),
    .m_axis_txwrite_data_TSTRB(txwr_tstrb), .m_axis_txwrite_data_TLAST(txwr_tlast),
    .m_axis_txwrite_data_TVALID(txwr_tvalid), .m_axis_txwrite_data_TREADY(txwr_tready),
    .s_axis_txread_data_TDATA(txrd_tdata), .s_axis_txread_data_TKEEP(txrd_tkeep),
    .s_axis_txread_data_TSTRB(txrd_tstrb), .s_axis_txread_data_TLAST(txrd_tlast),
    .s_axis_txread_data_TVALID(txrd_tvalid), .s_axis_txread_data_TREADY(txrd_tready),
    .s_axis_txwrite_sts_TDATA(txwr_sts_tdata), .s_axis_txwrite_sts_TVALID(txwr_sts_tvalid), .s_axis_txwrite_sts_TREADY(txwr_sts_tready),
    .m_axis_rxwrite_data_TDATA(rxwr_tdata), .m_axis_rxwrite_data_TKEEP(rxwr_tkeep),
    .m_axis_rxwrite_data_TSTRB(rxwr_tstrb), .m_axis_rxwrite_data_TLAST(rxwr_tlast),
    .m_axis_rxwrite_data_TVALID(rxwr_tvalid), .m_axis_rxwrite_data_TREADY(rxwr_tready),
    .s_axis_rxread_data_TDATA(rxrd_tdata), .s_axis_rxread_data_TKEEP(rxrd_tkeep),
    .s_axis_rxread_data_TSTRB(rxrd_tstrb), .s_axis_rxread_data_TLAST(rxrd_tlast),
    .s_axis_rxread_data_TVALID(rxrd_tvalid), .s_axis_rxread_data_TREADY(rxrd_tready),
`ifdef NO_RX_BYPASS
    .m_axis_rxwrite_cmd_TDATA(rxwr_cmd_tdata), .m_axis_rxwrite_cmd_TVALID(rxwr_cmd_tvalid), .m_axis_rxwrite_cmd_TREADY(rxwr_cmd_tready),
    .m_axis_rxread_cmd_TDATA(rxrd_cmd_tdata), .m_axis_rxread_cmd_TVALID(rxrd_cmd_tvalid), .m_axis_rxread_cmd_TREADY(rxrd_cmd_tready),
    .s_axis_rxwrite_sts_TDATA(rxwr_sts_tdata), .s_axis_rxwrite_sts_TVALID(rxwr_sts_tvalid), .s_axis_rxwrite_sts_TREADY(rxwr_sts_tready),
`else
    .axis_data_count(axis_data_count), .axis_max_data_count(axis_max_data_count),
`endif
    .m_axis_session_lup_req_TDATA(lup_req_tdata), .m_axis_session_lup_req_TVALID(lup_req_tvalid), .m_axis_session_lup_req_TREADY(lup_req_tready),
    .s_axis_session_lup_rsp_TDATA(lup_rsp_tdata), .s_axis_session_lup_rsp_TVALID(lup_rsp_tvalid), .s_axis_session_lup_rsp_TREADY(lup_rsp_tready),
    .m_axis_session_upd_req_TDATA(upd_req_tdata), .m_axis_session_upd_req_TVALID(upd_req_tvalid), .m_axis_session_upd_req_TREADY(upd_req_tready),
    .s_axis_session_upd_rsp_TDATA(upd_rsp_tdata), .s_axis_session_upd_rsp_TVALID(upd_rsp_tvalid), .s_axis_session_upd_rsp_TREADY(upd_rsp_tready),
    .s_axis_listen_port_req_TDATA(listen_req_tdata), .s_axis_listen_port_req_TVALID(listen_req_tvalid), .s_axis_listen_port_req_TREADY(listen_req_tready),
    .m_axis_listen_port_rsp_TDATA(listen_rsp_tdata), .m_axis_listen_port_rsp_TVALID(listen_rsp_tvalid), .m_axis_listen_port_rsp_TREADY(listen_rsp_tready),
    .m_axis_notification_TDATA(notif_tdata), .m_axis_notification_TVALID(notif_tvalid), .m_axis_notification_TREADY(notif_tready),
    .s_axis_rx_data_req_TDATA(rxreq_tdata), .s_axis_rx_data_req_TVALID(rxreq_tvalid), .s_axis_rx_data_req_TREADY(rxreq_tready),
    .m_axis_rx_data_rsp_metadata_TDATA(rxrsp_meta_tdata), .m_axis_rx_data_rsp_metadata_TVALID(rxrsp_meta_tvalid), .m_axis_rx_data_rsp_metadata_TREADY(rxrsp_meta_tready),
    .m_axis_rx_data_rsp_TDATA(rxrsp_tdata), .m_axis_rx_data_rsp_TKEEP(rxrsp_tkeep),
    .m_axis_rx_data_rsp_TSTRB(rxrsp_tstrb), .m_axis_rx_data_rsp_TLAST(rxrsp_tlast),
    .m_axis_rx_data_rsp_TVALID(rxrsp_tvalid), .m_axis_rx_data_rsp_TREADY(rxrsp_tready),
    .s_axis_open_conn_req_TDATA(open_req_tdata), .s_axis_open_conn_req_TVALID(open_req_tvalid), .s_axis_open_conn_req_TREADY(open_req_tready),
    .m_axis_open_conn_rsp_TDATA(open_rsp_tdata), .m_axis_open_conn_rsp_TVALID(open_rsp_tvalid), .m_axis_open_conn_rsp_TREADY(open_rsp_tready),
    .s_axis_close_conn_req_TDATA(close_req_tdata), .s_axis_close_conn_req_TVALID(close_req_tvalid), .s_axis_close_conn_req_TREADY(close_req_tready),
    .s_axis_tx_data_req_metadata_TDATA(txmeta_tdata), .s_axis_tx_data_req_metadata_TVALID(txmeta_tvalid), .s_axis_tx_data_req_metadata_TREADY(txmeta_tready),
    .s_axis_tx_data_req_TDATA(txdata_tdata), .s_axis_tx_data_req_TKEEP(txdata_tkeep),
    .s_axis_tx_data_req_TSTRB(txdata_tstrb), .s_axis_tx_data_req_TLAST(txdata_tlast),
    .s_axis_tx_data_req_TVALID(txdata_tvalid), .s_axis_tx_data_req_TREADY(txdata_tready),
    .m_axis_tx_data_rsp_TDATA(txrsp_tdata), .m_axis_tx_data_rsp_TVALID(txrsp_tvalid), .m_axis_tx_data_rsp_TREADY(txrsp_tready),
    .myIpAddress(ip_to_le(TOE_IP)),
    .regSessionCount(regSessionCount), .regSessionCount_ap_vld(regSessionCount_ap_vld)
  );

  //---------------------------------------------------------------------------
  // Bookkeeping / result
  //---------------------------------------------------------------------------
  int unsigned err_count = 0;
  string       testname  = "passive_open";

  task automatic tb_error(input string msg);
    err_count++;
    $display("[%0t] TB-ERROR: %s", $time, msg);
  endtask

  task automatic tb_info(input string msg);
    $display("[%0t] TB-INFO : %s", $time, msg);
  endtask

  function automatic void tb_check(input bit cond, input string msg);
    if (!cond) begin
      err_count++;
      $display("[%0t] TB-CHECK-FAIL: %s", $time, msg);
    end
    else begin
      $display("[%0t] TB-CHECK-OK  : %s", $time, msg);
    end
  endfunction

  //---------------------------------------------------------------------------
  // Session lookup CAM model
  // key = {theirPort[63:48], myPort[47:32], theirIp[31:0]} (LSB-first packing)
  //---------------------------------------------------------------------------
  bit [15:0] cam [bit [63:0]];
  int cam_delete_count = 0;
  int cam_insert_count = 0;

  // lookup server
  initial begin
    lup_req_tready <= 1;
    lup_rsp_tvalid <= 0;
    forever begin
      @(posedge ap_clk);
      if (ap_rst_n && lup_req_tvalid && lup_req_tready) begin
        bit [63:0] key; bit [31:0] src; bit [119:0] rsp;
        key = lup_req_tdata[63:0];
        src = lup_req_tdata[95:64];
        lup_req_tready <= 0;
        repeat (2) @(posedge ap_clk);
        rsp = '0;
        rsp[63:0] = key;
        if (cam.exists(key)) begin
          rsp[79:64] = cam[key];
          rsp[80]    = 1'b1;
        end
        rsp[112:81] = src;
        lup_rsp_tdata  <= rsp;
        lup_rsp_tvalid <= 1;
        do @(posedge ap_clk); while (!lup_rsp_tready);
        lup_rsp_tvalid <= 0;
        lup_req_tready <= 1;
        tb_info($sformatf("CAM LOOKUP key=%h hit=%0d id=%0d", key, rsp[80], rsp[79:64]));
      end
    end
  end

  // update server
  initial begin
    upd_req_tready <= 1;
    upd_rsp_tvalid <= 0;
    forever begin
      @(posedge ap_clk);
      if (ap_rst_n && upd_req_tvalid && upd_req_tready) begin
        bit [31:0] op, src; bit [63:0] key; bit [15:0] value; bit [151:0] rsp;
        op    = upd_req_tdata[31:0];
        key   = upd_req_tdata[95:32];
        value = upd_req_tdata[111:96];
        src   = upd_req_tdata[143:112];
        upd_req_tready <= 0;
        if (op == 0) begin cam[key] = value; cam_insert_count++; end   // INSERT
        else begin
          if (cam.exists(key)) cam.delete(key);                        // DELETE
          cam_delete_count++;
        end
        repeat (2) @(posedge ap_clk);
        rsp = '0;
        rsp[31:0]    = op;
        rsp[95:32]   = key;
        rsp[111:96]  = value;
        rsp[112]     = 1'b1;   // success
        rsp[144:113] = src;
        upd_rsp_tdata  <= rsp;
        upd_rsp_tvalid <= 1;
        do @(posedge ap_clk); while (!upd_rsp_tready);
        upd_rsp_tvalid <= 0;
        upd_req_tready <= 1;
        tb_info($sformatf("CAM %s key=%h id=%0d", (op==0)?"INSERT":"DELETE", key, value));
      end
    end
  end

  //---------------------------------------------------------------------------
  // TX buffer memory model (write cmd/data/status + read cmd/data)
  // mmCmd packing: bbt[22:0], type[23], dsa[29:24], eof[30], drr[31],
  //                saddr[63:32], tag[67:64], rsvd[71:68]
  //---------------------------------------------------------------------------
  byte unsigned tx_mem [longint unsigned];

  typedef struct packed { bit [31:0] saddr; bit [22:0] bbt; } memcmd_t;
  memcmd_t txwr_cmds[$];
  memcmd_t txrd_cmds[$];

  // capture commands
  always @(posedge ap_clk) begin
    if (ap_rst_n && txwr_cmd_tvalid && txwr_cmd_tready) begin
      memcmd_t c;
      c.bbt   = txwr_cmd_tdata[22:0];
      c.saddr = txwr_cmd_tdata[63:32];
      txwr_cmds.push_back(c);
    end
    if (ap_rst_n && txrd_cmd_tvalid && txrd_cmd_tready) begin
      memcmd_t c;
      c.bbt   = txrd_cmd_tdata[22:0];
      c.saddr = txrd_cmd_tdata[63:32];
      txrd_cmds.push_back(c);
    end
  end

  // write data path + status
  initial begin
    txwr_cmd_tready <= 1; txrd_cmd_tready <= 1;
    txwr_tready <= 0; txwr_sts_tvalid <= 0; txwr_sts_tdata <= 8'h80;
    forever begin
      memcmd_t c;
      longint unsigned addr;
      int unsigned remaining;
      while (txwr_cmds.size() == 0) @(posedge ap_clk);
      c = txwr_cmds.pop_front();
      addr = c.saddr; remaining = c.bbt;
      txwr_tready <= 1;
      while (remaining > 0) begin
        @(posedge ap_clk);
        if (txwr_tvalid && txwr_tready) begin
          for (int i = 0; i < 8; i++) begin
            if (remaining > 0 && txwr_tkeep[i]) begin
              tx_mem[addr] = txwr_tdata[8*i +: 8];
              addr++; remaining--;
            end
          end
          if (txwr_tlast) remaining = 0;
        end
      end
      txwr_tready <= 0;
      // status: okay=1 at bit7 (tag[3:0], interr, decerr, slverr, okay)
      txwr_sts_tdata  <= 8'h80;
      txwr_sts_tvalid <= 1;
      do @(posedge ap_clk); while (!txwr_sts_tready);
      txwr_sts_tvalid <= 0;
    end
  end

  // read data path
  initial begin
    txrd_tvalid <= 0; txrd_tdata <= '0; txrd_tkeep <= '0; txrd_tstrb <= '0; txrd_tlast <= 0;
    forever begin
      memcmd_t c;
      longint unsigned addr;
      int unsigned remaining, nbytes;
      bit [63:0] d; bit [7:0] k;
      while (txrd_cmds.size() == 0) @(posedge ap_clk);
      c = txrd_cmds.pop_front();
      addr = c.saddr; remaining = c.bbt;
      while (remaining > 0) begin
        nbytes = (remaining > 8) ? 8 : remaining;
        d = '0; k = '0;
        for (int i = 0; i < nbytes; i++) begin
          d[8*i +: 8] = tx_mem.exists(addr+i) ? tx_mem[addr+i] : 8'h00;
          k[i] = 1'b1;
        end
        txrd_tdata  <= d;
        txrd_tkeep  <= k;
        txrd_tstrb  <= k;
        txrd_tlast  <= (remaining <= 8);
        txrd_tvalid <= 1;
        do @(posedge ap_clk); while (!txrd_tready);
        addr += nbytes; remaining -= nbytes;
      end
      txrd_tvalid <= 0; txrd_tlast <= 0;
    end
  end

`ifdef NO_RX_BYPASS
  //---------------------------------------------------------------------------
  // RX buffer memory model (RX_DDR_BYPASS=0): write cmd/data/status + read
  // cmd/data, mirroring the TX buffer model
  //---------------------------------------------------------------------------
  byte unsigned rx_mem [longint unsigned];
  memcmd_t rx_wr_cmds[$];
  memcmd_t rx_rd_cmds[$];

  always @(posedge ap_clk) begin
    if (ap_rst_n && rxwr_cmd_tvalid && rxwr_cmd_tready) begin
      memcmd_t c;
      c.bbt   = rxwr_cmd_tdata[22:0];
      c.saddr = rxwr_cmd_tdata[63:32];
      rx_wr_cmds.push_back(c);
    end
    if (ap_rst_n && rxrd_cmd_tvalid && rxrd_cmd_tready) begin
      memcmd_t c;
      c.bbt   = rxrd_cmd_tdata[22:0];
      c.saddr = rxrd_cmd_tdata[63:32];
      rx_rd_cmds.push_back(c);
    end
  end

  // write data path + status
  initial begin
    rxwr_cmd_tready <= 1; rxrd_cmd_tready <= 1;
    rxwr_tready <= 0; rxwr_sts_tvalid <= 0; rxwr_sts_tdata <= 8'h80;
    forever begin
      memcmd_t c;
      longint unsigned addr;
      int unsigned remaining;
      while (rx_wr_cmds.size() == 0) @(posedge ap_clk);
      c = rx_wr_cmds.pop_front();
      addr = c.saddr; remaining = c.bbt;
      rxwr_tready <= 1;
      while (remaining > 0) begin
        @(posedge ap_clk);
        if (rxwr_tvalid && rxwr_tready) begin
          for (int i = 0; i < 8; i++) begin
            if (remaining > 0 && rxwr_tkeep[i]) begin
              rx_mem[addr] = rxwr_tdata[8*i +: 8];
              addr++; remaining--;
            end
          end
          if (rxwr_tlast) remaining = 0;
        end
      end
      rxwr_tready <= 0;
      rxwr_sts_tdata  <= 8'h80;   // okay
      rxwr_sts_tvalid <= 1;
      do @(posedge ap_clk); while (!rxwr_sts_tready);
      rxwr_sts_tvalid <= 0;
    end
  end

  // read data path
  initial begin
    rxrd_tvalid <= 0; rxrd_tdata <= '0; rxrd_tkeep <= '0; rxrd_tstrb <= '0; rxrd_tlast <= 0;
    forever begin
      memcmd_t c;
      longint unsigned addr;
      int unsigned remaining, nbytes;
      bit [63:0] d; bit [7:0] k;
      while (rx_rd_cmds.size() == 0) @(posedge ap_clk);
      c = rx_rd_cmds.pop_front();
      addr = c.saddr; remaining = c.bbt;
      while (remaining > 0) begin
        nbytes = (remaining > 8) ? 8 : remaining;
        d = '0; k = '0;
        for (int i = 0; i < nbytes; i++) begin
          d[8*i +: 8] = rx_mem.exists(addr+i) ? rx_mem[addr+i] : 8'h00;
          k[i] = 1'b1;
        end
        rxrd_tdata  <= d;
        rxrd_tkeep  <= k;
        rxrd_tstrb  <= k;
        rxrd_tlast  <= (remaining <= 8);
        rxrd_tvalid <= 1;
        do @(posedge ap_clk); while (!rxrd_tready);
        addr += nbytes; remaining -= nbytes;
      end
      rxrd_tvalid <= 0; rxrd_tlast <= 0;
    end
  end

  typedef struct packed { bit last; bit [7:0] keep; bit [63:0] data; } beat_t;
`else
  //---------------------------------------------------------------------------
  // RX bypass FIFO (rxwrite_data -> rxread_data), word-count exported
  //---------------------------------------------------------------------------
  typedef struct packed { bit last; bit [7:0] keep; bit [63:0] data; } beat_t;
  beat_t rx_fifo[$];

  initial begin
    rxwr_tready <= 1;
    rxrd_tvalid <= 0; rxrd_tdata <= '0; rxrd_tkeep <= '0; rxrd_tstrb <= '0; rxrd_tlast <= 0;
    axis_data_count <= 0;
  end

  always @(posedge ap_clk) begin
    if (ap_rst_n) begin
      if (rxwr_tvalid && rxwr_tready) begin
        beat_t b;
        b.data = rxwr_tdata; b.keep = rxwr_tkeep; b.last = rxwr_tlast;
        rx_fifo.push_back(b);
      end
      if (rxrd_tvalid && rxrd_tready)
        void'(rx_fifo.pop_front());
      if (rx_fifo.size() > 0) begin
        rxrd_tdata  <= rx_fifo[0].data;
        rxrd_tkeep  <= rx_fifo[0].keep;
        rxrd_tstrb  <= rx_fifo[0].keep;
        rxrd_tlast  <= rx_fifo[0].last;
        rxrd_tvalid <= 1;
      end
      else begin
        rxrd_tvalid <= 0;
      end
      axis_data_count <= rx_fifo.size();
    end
  end
`endif

  //---------------------------------------------------------------------------
  // App-side monitors: notifications / open responses / tx responses
  //---------------------------------------------------------------------------
  typedef struct packed {
    bit        closed;
    bit [15:0] dstPort;
    bit [31:0] ipAddress;
    bit [15:0] length;
    bit [15:0] sessionID;
  } notif_t;
  notif_t notif_q[$];

  typedef struct packed {
    bit [15:0] port;
    bit [31:0] ip;
    bit [7:0]  success;
    bit [15:0] sessionID;
  } openrsp_t;
  openrsp_t openrsp_q[$];

  typedef struct packed {
    bit [1:0]  error;
    bit [29:0] remaining_space;
    bit [15:0] length;
    bit [15:0] sessionID;
  } txrsp_t;
  txrsp_t txrsp_q[$];

  bit [15:0] rxmeta_q[$];
  bit        listen_rsp_q[$];

  initial begin
    notif_tready <= 1; open_rsp_tready <= 1; txrsp_tready <= 1;
    rxrsp_meta_tready <= 1; listen_rsp_tready <= 1;
  end

  always @(posedge ap_clk) begin
    if (ap_rst_n) begin
      if (notif_tvalid && notif_tready) begin
        notif_t n;
        n.sessionID = notif_tdata[15:0];
        n.length    = notif_tdata[31:16];
        n.ipAddress = notif_tdata[63:32];
        n.dstPort   = notif_tdata[79:64];
        n.closed    = notif_tdata[80];
        notif_q.push_back(n);
        tb_info($sformatf("APP notification: sid=%0d len=%0d ip=%h port=%0d closed=%0d",
                          n.sessionID, n.length, n.ipAddress, n.dstPort, n.closed));
      end
      if (open_rsp_tvalid && open_rsp_tready) begin
        openrsp_t o;
        o.sessionID = open_rsp_tdata[15:0];
        o.success   = open_rsp_tdata[23:16];
        o.ip        = open_rsp_tdata[55:24];
        o.port      = open_rsp_tdata[71:56];
        openrsp_q.push_back(o);
        tb_info($sformatf("APP openConnRsp: sid=%0d success=%0d", o.sessionID, o.success));
      end
      if (txrsp_tvalid && txrsp_tready) begin
        txrsp_t t;
        t.sessionID       = txrsp_tdata[15:0];
        t.length          = txrsp_tdata[31:16];
        t.remaining_space = txrsp_tdata[61:32];
        t.error           = txrsp_tdata[63:62];
        txrsp_q.push_back(t);
      end
      if (rxrsp_meta_tvalid && rxrsp_meta_tready)
        rxmeta_q.push_back(rxrsp_meta_tdata);
      if (listen_rsp_tvalid && listen_rsp_tready)
        listen_rsp_q.push_back(listen_rsp_tdata[0]);
    end
  end

  // rx data rsp collector
  byte unsigned app_rx_bytes[$];
  int unsigned  app_rx_pkts = 0;
  initial rxrsp_tready <= 1;
  always @(posedge ap_clk) begin
    if (ap_rst_n && rxrsp_tvalid && rxrsp_tready) begin
      for (int i = 0; i < 8; i++)
        if (rxrsp_tkeep[i]) app_rx_bytes.push_back(rxrsp_tdata[8*i +: 8]);
      if (rxrsp_tlast) app_rx_pkts++;
    end
  end

  //---------------------------------------------------------------------------
  // Network output monitor: reassemble packets from m_axis_tcp_data
  //---------------------------------------------------------------------------
  byte unsigned cur_pkt[$];
  bytes_t       net_pkts[$];   // queue of complete IPv4 packets

  initial m_tcp_tready <= 1;
  always @(posedge ap_clk) begin
    if (ap_rst_n && m_tcp_tvalid && m_tcp_tready) begin
      for (int i = 0; i < 8; i++)
        if (m_tcp_tkeep[i]) cur_pkt.push_back(m_tcp_tdata[8*i +: 8]);
      if (m_tcp_tlast) begin
        bytes_t p;
        p = new[cur_pkt.size()];
        foreach (cur_pkt[i]) p[i] = cur_pkt[i];
        cur_pkt.delete();
        net_pkts.push_back(p);
        print_pkt("DUT->NET", p);
        check_tcp_checksum(p);
      end
    end
  end

  //---------------------------------------------------------------------------
  // Packet field helpers (p = full IPv4 packet, big-endian on wire)
  //---------------------------------------------------------------------------
  function automatic bit [15:0] f16(input bytes_t p, input int off);
    return {p[off], p[off+1]};
  endfunction
  function automatic bit [31:0] f32(input bytes_t p, input int off);
    return {p[off], p[off+1], p[off+2], p[off+3]};
  endfunction
  function automatic bit [31:0] f_src_ip (input bytes_t p); return f32(p, 12); endfunction
  function automatic bit [31:0] f_dst_ip (input bytes_t p); return f32(p, 16); endfunction
  function automatic bit [15:0] f_src_port(input bytes_t p); return f16(p, 20); endfunction
  function automatic bit [15:0] f_dst_port(input bytes_t p); return f16(p, 22); endfunction
  function automatic bit [31:0] f_seq(input bytes_t p); return f32(p, 24); endfunction
  function automatic bit [31:0] f_ack(input bytes_t p); return f32(p, 28); endfunction
  function automatic bit [7:0]  f_flags(input bytes_t p); return p[33]; endfunction
  function automatic bit [15:0] f_win(input bytes_t p); return f16(p, 34); endfunction
  function automatic int f_data_off(input bytes_t p); return (p[32] >> 4) * 4; endfunction
  function automatic int f_payload_len(input bytes_t p);
    return f16(p, 2) - 20 - f_data_off(p);
  endfunction
  function automatic bytes_t f_payload(input bytes_t p);
    bytes_t d;
    int off, len;
    off = 20 + f_data_off(p);
    len = f_payload_len(p);
    d = new[len];
    for (int i = 0; i < len; i++) d[i] = p[off+i];
    return d;
  endfunction
  function automatic bytes_t f_options(input bytes_t p);
    bytes_t d;
    int len;
    len = f_data_off(p) - 20;
    d = new[len];
    for (int i = 0; i < len; i++) d[i] = p[40+i];
    return d;
  endfunction

  localparam bit [7:0] F_FIN = 8'h01, F_SYN = 8'h02, F_RST = 8'h04,
                       F_PSH = 8'h08, F_ACK = 8'h10;

  function automatic string flags_str(input bit [7:0] f);
    string s = "";
    if (f & F_SYN) s = {s, "SYN,"};
    if (f & F_ACK) s = {s, "ACK,"};
    if (f & F_FIN) s = {s, "FIN,"};
    if (f & F_RST) s = {s, "RST,"};
    if (f & F_PSH) s = {s, "PSH,"};
    return s;
  endfunction

  function automatic void print_pkt(input string dir, input bytes_t p);
    $display("[%0t] %s: %0d.%0d.%0d.%0d:%0d -> %0d.%0d.%0d.%0d:%0d [%s] seq=%0d ack=%0d win=%0d len=%0d",
      $time, dir,
      p[12], p[13], p[14], p[15], f_src_port(p),
      p[16], p[17], p[18], p[19], f_dst_port(p),
      flags_str(f_flags(p)), f_seq(p), f_ack(p), f_win(p), f_payload_len(p));
  endfunction

  //---------------------------------------------------------------------------
  // Checksum helpers
  //---------------------------------------------------------------------------
  function automatic bit [15:0] csum_finish(input bit [31:0] sum);
    bit [31:0] s = sum;
    while (s >> 16) s = (s & 32'hFFFF) + (s >> 16);
    return ~s[15:0];
  endfunction

  function automatic bit [15:0] ip_header_csum(input bytes_t p);
    bit [31:0] sum = 0;
    for (int i = 0; i < 20; i += 2)
      if (i != 10) sum += {p[i], p[i+1]};
    return csum_finish(sum);
  endfunction

  // TCP checksum over pseudo header + segment (checksum field skipped)
  function automatic bit [15:0] tcp_csum(input bytes_t p);
    bit [31:0] sum = 0;
    int tcp_len;
    tcp_len = f16(p, 2) - 20;
    sum += f16(p, 12); sum += f16(p, 14);   // src ip
    sum += f16(p, 16); sum += f16(p, 18);   // dst ip
    sum += 16'h0006;
    sum += tcp_len[15:0];
    for (int i = 0; i < tcp_len; i += 2) begin
      bit [15:0] w;
      w[15:8] = p[20+i];
      w[7:0]  = (i+1 < tcp_len) ? p[20+i+1] : 8'h00;
      if (i != 16) sum += w;   // tcp offset 16..17 = checksum field
    end
    return csum_finish(sum);
  endfunction

  function automatic void check_tcp_checksum(input bytes_t p);
    bit [15:0] expected = tcp_csum(p);
    bit [15:0] actual   = f16(p, 36);
    if (expected != actual) begin
      err_count++;
      $display("[%0t] TB-CHECK-FAIL: outgoing TCP checksum bad: got %h expected %h",
               $time, actual, expected);
    end
  endfunction

  //---------------------------------------------------------------------------
  // Packet builder (returns full IPv4 packet)
  //---------------------------------------------------------------------------
  function automatic bytes_t make_tcp(
      input bit [31:0] sip, input bit [31:0] dip,
      input bit [15:0] sp,  input bit [15:0] dp,
      input bit [31:0] seq, input bit [31:0] ack,
      input bit [7:0]  flags, input bit [15:0] win,
      input bytes_t opts, input bytes_t payload);
    bytes_t p;
    int optlen, hdrlen, totlen;
    bit [15:0] cs;
    optlen = (opts.size() + 3) / 4 * 4;
    hdrlen = 20 + optlen;
    totlen = 20 + hdrlen + payload.size();
    p = new[totlen];
    foreach (p[i]) p[i] = 8'h00;
    // IPv4 header
    // NOTE: concat-LHS assignment onto dynamic-array elements is broken in
    // xsim 2022.1 (only the first element gets written) - use per-byte stores.
    p[0] = 8'h45; p[1] = 8'h00;
    p[2] = totlen[15:8]; p[3] = totlen[7:0];
    p[8] = 8'h40;               // TTL
    p[9] = 8'h06;               // TCP
    p[12] = sip[31:24]; p[13] = sip[23:16]; p[14] = sip[15:8]; p[15] = sip[7:0];
    p[16] = dip[31:24]; p[17] = dip[23:16]; p[18] = dip[15:8]; p[19] = dip[7:0];
    cs = ip_header_csum(p);
    p[10] = cs[15:8]; p[11] = cs[7:0];
    // TCP header
    p[20] = sp[15:8]; p[21] = sp[7:0];
    p[22] = dp[15:8]; p[23] = dp[7:0];
    p[24] = seq[31:24]; p[25] = seq[23:16]; p[26] = seq[15:8]; p[27] = seq[7:0];
    p[28] = ack[31:24]; p[29] = ack[23:16]; p[30] = ack[15:8]; p[31] = ack[7:0];
    p[32] = (hdrlen / 4) << 4;
    p[33] = flags;
    p[34] = win[15:8]; p[35] = win[7:0];
    // options (pad with NOP=1)
    for (int i = 0; i < optlen; i++)
      p[40+i] = (i < opts.size()) ? opts[i] : 8'h01;
    // payload
    foreach (payload[i]) p[40+optlen+i] = payload[i];
    cs = tcp_csum(p);
    p[36] = cs[15:8]; p[37] = cs[7:0];
    return p;
  endfunction

  function automatic bytes_t no_bytes();
    bytes_t e; e = new[0]; return e;
  endfunction

  function automatic bytes_t syn_options_sack();
    bytes_t o;
    o = new[12];
    o[0] = 8'h02; o[1] = 8'h04; o[2] = 8'h05; o[3] = 8'hB4;  // MSS 1460
    o[4] = 8'h04; o[5] = 8'h02;                              // SACK permitted
    o[6] = 8'h03; o[7] = 8'h03; o[8] = 8'h02;                // WS shift=2
    o[9] = 8'h01; o[10] = 8'h01; o[11] = 8'h01;              // NOP padding
    return o;
  endfunction

  function automatic bytes_t syn_options(input bit with_ws);
    bytes_t o;
    if (with_ws) begin
      o = new[8];
      o[0] = 8'h02; o[1] = 8'h04; o[2] = 8'h05; o[3] = 8'hB4;  // MSS 1460
      o[4] = 8'h03; o[5] = 8'h03; o[6] = 8'h02; o[7] = 8'h01;  // WS shift=2 + NOP
    end
    else begin
      o = new[4];
      o[0] = 8'h02; o[1] = 8'h04; o[2] = 8'h05; o[3] = 8'hB4;
    end
    return o;
  endfunction

  // scan TCP options of packet p for `kind`; returns 1 if present and its
  // first value byte (byte at offset+2) in val (0 for 1/2-byte options)
  function automatic bit find_option(input bytes_t p, input byte unsigned kind,
                                     output byte unsigned val);
    bytes_t o;
    int i;
    o = f_options(p);
    i = 0; val = 0;
    while (i < o.size()) begin
      if (o[i] == 8'h00) return 0;          // EOL
      if (o[i] == kind) begin
        if (o[i] == 8'h01) begin val = 0; return 1; end
        if (i+2 < o.size()) val = o[i+2];
        return 1;
      end
      if (o[i] == 8'h01) i++;               // NOP
      else if (i+1 < o.size() && o[i+1] >= 2) i += o[i+1];
      else return 0;                        // malformed
    end
    return 0;
  endfunction

  // extract the first SACK block (kind 5) from packet p
  function automatic bit get_sack_block(input bytes_t p,
                                        output bit [31:0] sstart,
                                        output bit [31:0] send_);
    bytes_t o;
    int i;
    o = f_options(p);
    i = 0; sstart = 0; send_ = 0;
    while (i < o.size()) begin
      if (o[i] == 8'h00) return 0;
      if (o[i] == 8'h01) begin i++; continue; end
      if (o[i] == 8'h05) begin
        if (i+9 < o.size()) begin
          sstart = {o[i+2], o[i+3], o[i+4], o[i+5]};
          send_  = {o[i+6], o[i+7], o[i+8], o[i+9]};
          return 1;
        end
        return 0;
      end
      if (i+1 < o.size() && o[i+1] >= 2) i += o[i+1];
      else return 0;
    end
    return 0;
  endfunction

  // extract up to two SACK blocks (kind 5); returns the block count (0/1/2)
  function automatic int get_sack_blocks(input bytes_t p,
                                         output bit [31:0] s0, output bit [31:0] e0,
                                         output bit [31:0] s1, output bit [31:0] e1);
    bytes_t o;
    int i;
    o = f_options(p);
    i = 0; s0 = 0; e0 = 0; s1 = 0; e1 = 0;
    while (i < o.size()) begin
      if (o[i] == 8'h00) return 0;
      if (o[i] == 8'h01) begin i++; continue; end
      if (o[i] == 8'h05) begin
        if (i+9 < o.size()) begin
          s0 = {o[i+2], o[i+3], o[i+4], o[i+5]};
          e0 = {o[i+6], o[i+7], o[i+8], o[i+9]};
          if (o[i+1] >= 8'h12 && i+17 < o.size()) begin
            s1 = {o[i+10], o[i+11], o[i+12], o[i+13]};
            e1 = {o[i+14], o[i+15], o[i+16], o[i+17]};
            return 2;
          end
          return 1;
        end
        return 0;
      end
      if (i+1 < o.size() && o[i+1] >= 2) i += o[i+1];
      else return 0;
    end
    return 0;
  endfunction

  // 12-byte option list with one SACK block: NOP NOP kind=5 len=10 start end
  function automatic bytes_t sack_opt(input bit [31:0] sstart, input bit [31:0] send_);
    bytes_t o;
    o = new[12];
    o[0] = 8'h01; o[1] = 8'h01; o[2] = 8'h05; o[3] = 8'h0A;
    o[4] = sstart[31:24]; o[5] = sstart[23:16]; o[6] = sstart[15:8]; o[7] = sstart[7:0];
    o[8] = send_[31:24];  o[9] = send_[23:16]; o[10] = send_[15:8]; o[11] = send_[7:0];
    return o;
  endfunction

  // 20-byte SYN option list with timestamps:
  // MSS | SACKperm | TS(kind8 len10 tsval tsecr=0) | WS shift=2 | NOP
  function automatic bytes_t ts_syn_options(input bit [31:0] tsval);
    bytes_t o;
    o = new[20];
    o[0] = 8'h02; o[1] = 8'h04; o[2] = 8'h05; o[3] = 8'hB4;   // MSS 1460
    o[4] = 8'h04; o[5] = 8'h02;                               // SACK permitted
    o[6] = 8'h08; o[7] = 8'h0A;                               // TS kind/len
    o[8]  = tsval[31:24]; o[9]  = tsval[23:16]; o[10] = tsval[15:8]; o[11] = tsval[7:0];
    o[12] = 8'h00; o[13] = 8'h00; o[14] = 8'h00; o[15] = 8'h00; // TSecr = 0
    o[16] = 8'h03; o[17] = 8'h03; o[18] = 8'h02;              // WS shift=2
    o[19] = 8'h01;                                            // NOP pad
    return o;
  endfunction

  // 12-byte option list: NOP NOP TS(kind8 len10 tsval tsecr)
  function automatic bytes_t ts_opt(input bit [31:0] tsval, input bit [31:0] tsecr);
    bytes_t o;
    o = new[12];
    o[0] = 8'h01; o[1] = 8'h01; o[2] = 8'h08; o[3] = 8'h0A;
    o[4] = tsval[31:24]; o[5] = tsval[23:16]; o[6] = tsval[15:8]; o[7] = tsval[7:0];
    o[8] = tsecr[31:24]; o[9] = tsecr[23:16]; o[10] = tsecr[15:8]; o[11] = tsecr[7:0];
    return o;
  endfunction

  // extract the timestamps option (kind 8) from packet p
  function automatic bit get_ts_option(input bytes_t p,
                                       output bit [31:0] tsval,
                                       output bit [31:0] tsecr);
    bytes_t o;
    int i;
    o = f_options(p);
    i = 0; tsval = 0; tsecr = 0;
    while (i < o.size()) begin
      if (o[i] == 8'h00) return 0;
      if (o[i] == 8'h01) begin i++; continue; end
      if (o[i] == 8'h08) begin
        if (i+9 < o.size()) begin
          tsval = {o[i+2], o[i+3], o[i+4], o[i+5]};
          tsecr = {o[i+6], o[i+7], o[i+8], o[i+9]};
          return 1;
        end
        return 0;
      end
      if (i+1 < o.size() && o[i+1] >= 2) i += o[i+1];
      else return 0;
    end
    return 0;
  endfunction

  function automatic bytes_t gen_payload(input int len, input byte unsigned seed);
    bytes_t d;
    d = new[len];
    for (int i = 0; i < len; i++) d[i] = (seed + i) & 8'hFF;
    return d;
  endfunction

  //---------------------------------------------------------------------------
  // Network input driver
  //---------------------------------------------------------------------------
  task automatic net_send(input bytes_t p);
    int nbeats;
    nbeats = (p.size() + 7) / 8;
    print_pkt("NET->DUT", p);
    for (int b = 0; b < nbeats; b++) begin
      bit [63:0] d; bit [7:0] k;
      d = '0; k = '0;
      for (int i = 0; i < 8; i++) begin
        int idx;
        idx = b*8 + i;
        if (idx < p.size()) begin
          d[8*i +: 8] = p[idx];
          k[i] = 1'b1;
        end
      end
      s_tcp_tdata <= d; s_tcp_tkeep <= k; s_tcp_tstrb <= k;
      s_tcp_tlast <= (b == nbeats-1);
      s_tcp_tvalid <= 1;
      do @(posedge ap_clk); while (!s_tcp_tready);
    end
    s_tcp_tvalid <= 0; s_tcp_tlast <= 0;
    tb_info("net_send done");
  endtask

  //---------------------------------------------------------------------------
  // Wait / expect helpers
  //---------------------------------------------------------------------------
  task automatic wait_pkt(output bytes_t p, input int timeout_cycles, output bit ok);
    int n;
    n = 0; ok = 0;
    while (n < timeout_cycles) begin
      @(posedge ap_clk);
      n++;
      if (net_pkts.size() > 0) begin
        p = net_pkts.pop_front();
        ok = 1;
        return;
      end
    end
    p = no_bytes();
  endtask

  // wait for a packet with all `must` flags set; optionally skip pure ACKs
  task automatic expect_flags(input bit [7:0] must, input string what,
                              output bytes_t p, input int timeout_cycles,
                              input bit skip_pure_ack);
    bit ok;
    forever begin
      wait_pkt(p, timeout_cycles, ok);
      if (!ok) begin
        tb_error($sformatf("timeout waiting for %s", what));
        return;
      end
      if ((f_flags(p) & must) == must) begin
        tb_info($sformatf("got expected %s", what));
        return;
      end
      if (skip_pure_ack && (f_flags(p) == F_ACK) && f_payload_len(p) == 0) begin
        tb_info($sformatf("(skipping pure ACK while waiting for %s)", what));
        continue;
      end
      tb_error($sformatf("expected %s, got flags=%s", what, flags_str(f_flags(p))));
      return;
    end
  endtask

  task automatic wait_notif(output notif_t n, input int timeout_cycles, output bit ok);
    int cyc;
    cyc = 0; ok = 0;
    while (cyc < timeout_cycles) begin
      @(posedge ap_clk);
      cyc++;
      if (notif_q.size() > 0) begin
        n = notif_q.pop_front();
        ok = 1;
        return;
      end
    end
  endtask

  //---------------------------------------------------------------------------
  // App BFM tasks
  //---------------------------------------------------------------------------
  task automatic app_listen(input bit [15:0] port, output bit ok);
    int n;
    listen_req_tdata <= port; listen_req_tvalid <= 1;
    do @(posedge ap_clk); while (!listen_req_tready);
    listen_req_tvalid <= 0;
    ok = 0; n = 0;
    while (n < 10000) begin
      @(posedge ap_clk); n++;
      if (listen_rsp_q.size() > 0) begin
        ok = listen_rsp_q.pop_front();
        return;
      end
    end
    tb_error("timeout waiting for listen response");
  endtask

  task automatic app_read(input bit [15:0] sid, input int len,
                          output byte unsigned data[$]);
    int n, start_bytes;
    n = 0;
    start_bytes = app_rx_bytes.size();
    rxreq_tdata[15:0]  <= sid;
    rxreq_tdata[31:16] <= len[15:0];
    rxreq_tvalid <= 1;
    do @(posedge ap_clk); while (!rxreq_tready);
    rxreq_tvalid <= 0;
    while (app_rx_bytes.size() < start_bytes + len && n < 100000) begin
      @(posedge ap_clk); n++;
    end
    if (app_rx_bytes.size() < start_bytes + len)
      tb_error($sformatf("app_read timeout: wanted %0d bytes, got %0d",
               len, app_rx_bytes.size() - start_bytes));
    data.delete();
    for (int i = start_bytes; i < app_rx_bytes.size(); i++)
      data.push_back(app_rx_bytes[i]);
  endtask

  // returns response (error==0 -> data actually sent)
  task automatic app_send(input bit [15:0] sid, input bytes_t data,
                          output txrsp_t rsp);
    int n;
    n = 0;
    txmeta_tdata[15:0]  <= sid;
    txmeta_tdata[31:16] <= data.size();
    txmeta_tvalid <= 1;
    do @(posedge ap_clk); while (!txmeta_tready);
    txmeta_tvalid <= 0;
    while (txrsp_q.size() == 0 && n < 100000) begin
      @(posedge ap_clk); n++;
    end
    if (txrsp_q.size() == 0) begin
      tb_error("app_send: timeout waiting for txDataRsp");
      rsp = '0; rsp.error = 2'b11;
      return;
    end
    rsp = txrsp_q.pop_front();
    if (rsp.error != 0) return;
    begin
      int nbeats;
      nbeats = (data.size() + 7) / 8;
      for (int b = 0; b < nbeats; b++) begin
        bit [63:0] d; bit [7:0] k;
        d = '0; k = '0;
        for (int i = 0; i < 8; i++) begin
          int idx;
          idx = b*8 + i;
          if (idx < data.size()) begin
            d[8*i +: 8] = data[idx];
            k[i] = 1'b1;
          end
        end
        txdata_tdata <= d; txdata_tkeep <= k; txdata_tstrb <= k;
        txdata_tlast <= (b == nbeats-1);
        txdata_tvalid <= 1;
        do @(posedge ap_clk); while (!txdata_tready);
      end
      txdata_tvalid <= 0; txdata_tlast <= 0;
    end
  endtask

  task automatic app_open(input bit [31:0] ip_le, input bit [15:0] port);
    open_req_tdata[31:0]  <= ip_le;
    open_req_tdata[47:32] <= port;
    open_req_tvalid <= 1;
    do @(posedge ap_clk); while (!open_req_tready);
    open_req_tvalid <= 0;
  endtask

  task automatic app_close(input bit [15:0] sid);
    close_req_tdata <= sid; close_req_tvalid <= 1;
    do @(posedge ap_clk); while (!close_req_tready);
    close_req_tvalid <= 0;
  endtask

  //---------------------------------------------------------------------------
  // Peer connection state (single connection helper for tests)
  //---------------------------------------------------------------------------
  bit [31:0] peer_seq;      // next seq we will send
  bit [31:0] peer_ack;      // next expected byte from TOE (our ack)
  bit [15:0] peer_port;
  bit [15:0] toe_port;
  bit [15:0] peer_win = 16'hFFFF;

  task automatic peer_send_seg(input bit [7:0] flags, input bytes_t payload);
    bytes_t p;
    p = make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                 peer_seq, peer_ack, flags, peer_win, no_bytes(), payload);
    net_send(p);
  endtask

  // Passive-open handshake (TOE is server). Leaves peer_seq/peer_ack set.
  task automatic do_passive_handshake(input bit [15:0] lport, input bit [15:0] rport,
                                      input bit [31:0] init_seq = 32'h1000_0000);
    bytes_t p;
    bit [31:0] toe_iss;
    peer_port = rport; toe_port = lport;
    peer_seq  = init_seq;
    peer_win  = 16'hFFFF;
    // SYN ->
    p = make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                 peer_seq, 0, F_SYN, 16'hFFFF, syn_options(1), no_bytes());
    net_send(p);
    // <- SYN-ACK (skip stray pure ACKs, e.g. window updates of earlier sessions)
    expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 50000, 1);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq + 1, $sformatf("SYN-ACK ack=%0d expected %0d", f_ack(p), peer_seq+1));
    tb_check(f_src_port(p) == lport, "SYN-ACK src port");
    tb_check(f_dst_port(p) == rport, "SYN-ACK dst port");
    toe_iss  = f_seq(p);
    peer_ack = toe_iss + 1;
    peer_seq = peer_seq + 1;
    // ACK ->
    peer_send_seg(F_ACK, no_bytes());
    tb_info($sformatf("handshake complete, TOE ISS=%0d", toe_iss));
  endtask

  // passive handshake with RFC 7323 timestamps; peer_tsval is the TSval the
  // peer used on its SYN (subsequent segments should use larger values)
  task automatic do_ts_handshake(input bit [15:0] lport, input bit [15:0] rport,
                                 input bit [31:0] init_seq, input bit [31:0] peer_tsval,
                                 output bit [31:0] toe_tsval);
    bytes_t p;
    bit [31:0] toe_iss, tv, te;
    bit ok;
    peer_port = rport; toe_port = lport;
    peer_seq  = init_seq;
    peer_win  = 16'hFFFF;
    p = make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                 peer_seq, 0, F_SYN, 16'hFFFF, ts_syn_options(peer_tsval), no_bytes());
    net_send(p);
    expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 50000, 1);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq + 1, $sformatf("SYN-ACK ack=%0d expected %0d", f_ack(p), peer_seq+1));
    ok = get_ts_option(p, tv, te);
    tb_check(ok, "SYN-ACK carries a timestamps option");
    if (ok) begin
      tb_check(te == peer_tsval, $sformatf("SYN-ACK TSecr=%0d expected %0d", te, peer_tsval));
    end
    toe_tsval = tv;
    toe_iss  = f_seq(p);
    peer_ack = toe_iss + 1;
    peer_seq = peer_seq + 1;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, peer_ack, F_ACK, peer_win,
                      ts_opt(peer_tsval + 1, toe_tsval), no_bytes()));
    tb_info($sformatf("TS handshake complete, TOE ISS=%0d TSval=%0d", toe_iss, toe_tsval));
  endtask

  //---------------------------------------------------------------------------
  // Tests
  //---------------------------------------------------------------------------
  task automatic test_passive_open();
    bytes_t p, pay;
    bit ok;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80) accepted");

    do_passive_handshake(16'd80, 16'd41000);
    if (err_count) return;

    // --- peer sends 32B data ---
    pay = gen_payload(32, 8'h10);
    peer_send_seg(F_ACK, pay);
    wait_notif(n, 100000, ok);
    tb_check(ok, "data notification received");
    if (!ok) return;
    tb_check(n.length == 32, $sformatf("notification length=%0d expected 32", n.length));
    tb_check(n.dstPort == 80, "notification dst port");
    sid = n.sessionID;
    peer_seq += 32;

    // app reads the data
    app_read(sid, 32, rdata);
    tb_check(rdata.size() == 32, "app_read got 32 bytes");
    for (int i = 0; i < 32 && i < rdata.size(); i++)
      if (rdata[i] != pay[i]) begin
        tb_check(0, $sformatf("payload byte %0d mismatch: %h != %h", i, rdata[i], pay[i]));
        break;
      end

    // expect (delayed) ACK covering the data
    expect_flags(F_ACK, "ACK of data", p, 300000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq, $sformatf("data ACK ack=%0d expected %0d", f_ack(p), peer_seq));

    // --- peer closes: FIN ---
    peer_send_seg(F_FIN | F_ACK, no_bytes());
    peer_seq += 1;
    // TOE notifies app of close
    wait_notif(n, 300000, ok);
    tb_check(ok && n.closed, "close notification");
    // half-close: TOE ACKs the FIN and stays in CLOSE_WAIT
    expect_flags(F_ACK, "ACK of peer FIN", p, 400000, 0);
    if (err_count) return;
    tb_check((f_flags(p) & F_FIN) == 0, "no premature FIN from TOE (CLOSE_WAIT)");
    tb_check(f_ack(p) == peer_seq, $sformatf("FIN ACK ack=%0d expected %0d", f_ack(p), peer_seq));
    // app closes -> now the TOE sends its FIN
    app_close(sid);
    expect_flags(F_FIN, "FIN after app close", p, 400000, 1);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq, $sformatf("FIN ack=%0d expected %0d", f_ack(p), peer_seq));
    peer_ack = f_seq(p) + f_payload_len(p) + 1;
    // final ACK ->
    peer_send_seg(F_ACK, no_bytes());
    repeat (2000) @(posedge ap_clk);
  endtask

  task automatic peer_ack_fin(input bytes_t fin_pkt);
    bytes_t p;
    // ACK of TOE FIN
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, f_seq(fin_pkt) + 1, F_ACK, peer_win,
                      no_bytes(), no_bytes()));
    // our FIN
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, f_seq(fin_pkt) + 1, F_FIN | F_ACK, peer_win,
                      no_bytes(), no_bytes()));
    peer_seq += 1;
    // TOE should ACK our FIN (goes to TIME_WAIT)
    expect_flags(F_ACK, "ACK of peer FIN (TIME_WAIT)", p, 400000, 0);
    if (!err_count)
      tb_check(f_ack(p) == peer_seq, $sformatf("TIME_WAIT ack=%0d expected %0d", f_ack(p), peer_seq));
  endtask

  task automatic test_active_open();
    bytes_t p, pay;
    bit ok;
    bit [31:0] toe_iss;
    bit [15:0] sid;
    txrsp_t rsp;
    openrsp_t orsp;
    int n;

    // app initiates connection to PEER_IP:60000
    // (openConnReq takes the IP in display/big-endian order, the port in host order)
    app_open(PEER_IP, 16'd60000);
    // <- SYN
    expect_flags(F_SYN, "SYN", p, 100000, 0);
    if (err_count) return;
    tb_check((f_flags(p) & F_ACK) == 0, "SYN has no ACK flag");
    tb_check(f_dst_ip(p) == PEER_IP, $sformatf("SYN dst ip %h expected %h", f_dst_ip(p), PEER_IP));
    tb_check(f_dst_port(p) == 16'd60000, $sformatf("SYN dst port %0d expected 60000", f_dst_port(p)));
    // check options: MSS + window scale expected
    begin
      bytes_t o;
      bit seen_mss, seen_ws;
      int i;
      o = f_options(p);
      seen_mss = 0; seen_ws = 0; i = 0;
      while (i < o.size()) begin
        if (o[i] == 8'h00) break;
        else if (o[i] == 8'h01) i++;
        else if (o[i] == 8'h02) begin
          seen_mss = 1;
          tb_check({o[i+2], o[i+3]} == MSS[15:0], "SYN MSS option value");
          i += 4;
        end
        else if (o[i] == 8'h03) begin seen_ws = 1; i += 3; end
        else i += (i+1 < o.size() && o[i+1] > 0) ? o[i+1] : 1;
      end
      tb_check(seen_mss, "SYN carries MSS option");
      tb_check(seen_ws, "SYN carries window-scale option");
    end
    toe_iss   = f_seq(p);
    toe_port  = f_src_port(p);
    peer_port = 16'd60000;
    peer_seq  = 32'h2000_0000;
    peer_ack  = toe_iss + 1;
    peer_win  = 16'hFFFF;
    // SYN-ACK ->
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, peer_ack, F_SYN | F_ACK, 16'hFFFF,
                      syn_options(1), no_bytes()));
    peer_seq += 1;
    // <- ACK
    expect_flags(F_ACK, "final ACK of 3WHS", p, 100000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq, "3WHS ACK number");
    // open response to app
    n = 0;
    while (openrsp_q.size() == 0 && n < 50000) begin @(posedge ap_clk); n++; end
    tb_check(openrsp_q.size() > 0, "openConnRsp received");
    if (openrsp_q.size() == 0) return;
    orsp = openrsp_q.pop_front();
    tb_check(orsp.success != 0, "openConnRsp success");
    sid = orsp.sessionID;

    // --- app sends 64B ---
    pay = gen_payload(64, 8'hA0);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, $sformatf("app_send accepted (err=%0d)", rsp.error));
    expect_flags(F_ACK, "data segment", p, 300000, 0);
    if (err_count) return;
    tb_check(f_payload_len(p) == 64, $sformatf("data len=%0d expected 64", f_payload_len(p)));
    tb_check(f_seq(p) == peer_ack, "data seq");
    begin
      bytes_t d;
      d = f_payload(p);
      for (int i = 0; i < 64 && i < d.size(); i++)
        if (d[i] != pay[i]) begin
          tb_check(0, $sformatf("tx payload byte %0d mismatch %h != %h", i, d[i], pay[i]));
          break;
        end
    end
    peer_ack += 64;
    peer_send_seg(F_ACK, no_bytes());

    // --- app closes ---
    app_close(sid);
    expect_flags(F_FIN, "FIN after app close", p, 300000, 1);
    if (err_count) return;
    tb_check(f_seq(p) == peer_ack, "FIN seq");
    peer_ack_fin(p);
    repeat (2000) @(posedge ap_clk);
  endtask

  task automatic test_rx_out_of_order();
    bytes_t p, payA, payB;
    bit ok;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    bit [31:0] base_seq;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");
    do_passive_handshake(16'd80, 16'd41001);
    if (err_count) return;
    base_seq = peer_seq;

    payA = gen_payload(1000, 8'h20);
    payB = gen_payload(500, 8'h60);

`ifdef NO_RX_BYPASS
    // --- RX_DDR_BYPASS=0: single-gap OOO buffering ---
    // send segment B first (seq skips 1000 bytes) -> buffered as premature
    // data in DDR; dup-ACK sent; the internal length-0 notification is
    // filtered by rxAppNotificationDelayer, so the app sees nothing yet
    peer_seq = base_seq + 1000;
    peer_send_seg(F_ACK, payB);
    expect_flags(F_ACK, "dup ACK for OOO segment", p, 100000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == base_seq, $sformatf("dup ACK ack=%0d expected %0d (gap open)", f_ack(p), base_seq));
    tb_check(notif_q.size() == 0, "no app notification for buffered OOO segment");

    // segment A fills the gap exactly -> one notification covering A+B (1500B)
    peer_seq = base_seq;
    peer_send_seg(F_ACK, payA);
    wait_notif(n, 100000, ok);
    tb_check(ok && n.length == 1500, $sformatf("gap-fill notified len=%0d expected 1500", n.length));
    if (!ok) return;
    sid = n.sessionID;
    peer_seq = base_seq + 1500;

    app_read(sid, 1500, rdata);
    tb_check(rdata.size() == 1500, "app_read got 1500 bytes");
    for (int i = 0; i < 1000 && i < rdata.size(); i++)
      if (rdata[i] != payA[i]) begin tb_check(0, "segment A payload mismatch"); break; end
    for (int i = 0; i < 500 && (1000+i) < rdata.size(); i++)
      if (rdata[1000+i] != payB[i]) begin tb_check(0, "segment B payload mismatch"); break; end
`else
    // --- RX_DDR_BYPASS=1: OOO segments are dropped ---
    // send segment B first (seq skips 1000 bytes) -> dropped in bypass mode
    peer_seq = base_seq + 1000;
    peer_send_seg(F_ACK, payB);
    // expect immediate dup-ACK with ack == base_seq (nothing accepted)
    expect_flags(F_ACK, "dup ACK for OOO segment", p, 100000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == base_seq, $sformatf("dup ACK ack=%0d expected %0d (OOO dropped)", f_ack(p), base_seq));
    tb_check(notif_q.size() == 0, "no app notification for OOO segment");

    // now send segment A in order -> accepted
    peer_seq = base_seq;
    peer_send_seg(F_ACK, payA);
    wait_notif(n, 100000, ok);
    tb_check(ok && n.length == 1000, "in-order segment A notified");
    if (!ok) return;
    sid = n.sessionID;

    // resend segment B, now in order -> accepted
    peer_seq = base_seq + 1000;
    peer_send_seg(F_ACK, payB);
    wait_notif(n, 100000, ok);
    tb_check(ok && n.length == 500, "retransmitted segment B notified");
    peer_seq = base_seq + 1500;

    app_read(sid, 1000, rdata);
    for (int i = 0; i < 1000 && i < rdata.size(); i++)
      if (rdata[i] != payA[i]) begin tb_check(0, "segment A payload mismatch"); break; end
    app_read(sid, 500, rdata);
    for (int i = 0; i < 500 && i < rdata.size(); i++)
      if (rdata[i] != payB[i]) begin tb_check(0, "segment B payload mismatch"); break; end
`endif
    repeat (2000) @(posedge ap_clk);
  endtask

  task automatic test_fast_retransmit();
    bytes_t p, pay;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] d0_seq, d0_end;
    int got;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41002);
    if (err_count) return;

    // learn session id: peer sends 8B first
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 100000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;

    // app sends 5 x 1000B
    for (int i = 0; i < 5; i++) begin
      pay = gen_payload(1000, 8'h30 + i[7:0]);
      app_send(sid, pay, rsp);
      tb_check(rsp.error == 0, $sformatf("app_send %0d accepted", i));
    end
    // collect the 5 data segments (skip delayed ACK of the 8B)
    got = 0; d0_seq = 0;
    while (got < 5) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout collecting data segments"); return; end
      if (f_payload_len(p) == 1000) begin
        if (got == 0) d0_seq = f_seq(p);
        got++;
      end
    end
    d0_end = d0_seq + 1000;

    // 1 regular ACK for first segment, then 3 duplicate ACKs
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_end, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (100) @(posedge ap_clk);
    for (int i = 0; i < 3; i++) begin
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, d0_end, F_ACK, peer_win, no_bytes(), no_bytes()));
      repeat (50) @(posedge ap_clk);
    end

    // expect fast retransmission starting at d0_end (go-back-N)
    got = 0;
    while (got == 0) begin
      wait_pkt(p, 500000, ok);
      if (!ok) begin tb_error("timeout waiting for fast retransmit"); return; end
      if (f_payload_len(p) > 0) begin
        tb_check(f_seq(p) == d0_end,
          $sformatf("fast-rtx seq=%0d expected %0d", f_seq(p), d0_end));
        got = 1;
      end
    end
    // ACK everything to clean up
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_seq + 5000, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
  endtask

  task automatic test_rst_handling();
    bytes_t p;
    bit ok;
    notif_t n;

    // 1) data segment to a port nobody listens on -> RST from TOE
    net_send(make_tcp(PEER_IP, TOE_IP, 16'd50000, 16'd12345,
                      32'h0300_0000, 32'h0, F_ACK, 16'hFFFF,
                      no_bytes(), gen_payload(4, 8'h55)));
    expect_flags(F_RST, "RST for closed port", p, 100000, 0);
    if (err_count) return;

    // 2) established session + RST with seq == recvd -> session closed, app notified
    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41003);
    if (err_count) return;
    peer_send_seg(F_RST, no_bytes());
    wait_notif(n, 300000, ok);
    tb_check(ok && n.closed, "app notified of RST close");
    repeat (2000) @(posedge ap_clk);
  endtask

  task automatic test_zero_window();
    bytes_t p, pay;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41004);
    if (err_count) return;

    // learn session id
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 100000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;

    // consume the delayed ACK of the 8B data
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // peer advertises zero window
    peer_win = 16'h0000;
    peer_send_seg(F_ACK, no_bytes());
    repeat (5000) @(posedge ap_clk);

    // app tries to send -> must be rejected (no window)
    pay = gen_payload(1000, 8'h77);
    app_send(sid, pay, rsp);
    tb_check(rsp.error != 0, $sformatf("send with zero window rejected (err=%0d)", rsp.error));
    begin
      wait_pkt(p, 30000, ok);
      if (ok && f_payload_len(p) > 0)
        tb_check(0, "TOE sent data despite zero window");
    end

    // window update
    peer_win = 16'h4000;
    peer_send_seg(F_ACK, no_bytes());
    repeat (2000) @(posedge ap_clk);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, $sformatf("send after window update accepted (err=%0d)", rsp.error));
    // skip pure ACKs (e.g. the persist-resume probe fired on the window update)
    begin
      bit got_data = 0;
      while (!got_data) begin
        wait_pkt(p, 400000, ok);
        if (!ok) begin tb_error("timeout waiting for data after window update"); return; end
        if (f_payload_len(p) != 0) got_data = 1;
      end
    end
    tb_check(f_payload_len(p) == 1000, "1000B data segment after window opened");
    peer_win = 16'hFFFF;
    repeat (2000) @(posedge ap_clk);
  endtask

  // B4 regression (issue #31 + rxAppMemAccessBreakdown 64KB bug): memory
  // accesses that wrap the per-session 256KB circular buffer (or merely cross
  // a 64KB boundary on the legacy rx-app read splitter) must return correct
  // data.
  localparam int BUF_SIZE = 1 << 18;   // WINDOW_BITS = 18

  task automatic wrap_rx_subtest(input bit [15:0] rport, input bit [31:0] init_seq,
                                 input int dlen, input string what);
    bytes_t pay;
    bit ok;
    notif_t n;
    byte unsigned rdata[$];
    do_passive_handshake(16'd80, rport, init_seq);
    if (err_count) return;
    pay = gen_payload(dlen, 8'h33);
    peer_send_seg(F_ACK, pay);
    wait_notif(n, 100000, ok);
    tb_check(ok && n.length == dlen[15:0], {what, ": data notification"});
    if (!ok) return;
    peer_seq += dlen;
    app_read(n.sessionID, dlen, rdata);
    tb_check(rdata.size() == dlen, {what, ": read length"});
    for (int i = 0; i < dlen && i < rdata.size(); i++)
      if (rdata[i] != pay[i]) begin
        tb_check(0, $sformatf("%s: payload mismatch at byte %0d (%h != %h)",
                              what, i, rdata[i], pay[i]));
        break;
      end
    tb_info({what, ": data verified"});
  endtask

  task automatic test_buffer_wrap();
    bytes_t p;
    bit ok, found;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    int tries;
    bit [31:0] iss;
    bit [17:0] off;
    byte unsigned sent[$];
    bit [31:0] data_base;
    int got, rt_bytes;
    bytes_t pay;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");

`ifdef NO_RX_BYPASS
    // --- RX side: peer seq crossing the 256KB buffer wrap ---
    wrap_rx_subtest(16'd42000, 32'h0003FF69, 400, "rx 256KB wrap");
    if (err_count) return;
    // --- RX side: crossing only a 64KB boundary (legacy splitter bug) ---
    wrap_rx_subtest(16'd42001, 32'h0000FF00, 400, "rx 64KB crossing");
    if (err_count) return;
`endif

    // --- TX side: find a connection whose TOE ISS lands just below the
    // 256KB boundary, then trigger go-back-N retransmission across the wrap ---
    tries = 0; found = 0;
    while (tries < 400 && !found) begin
      peer_port = 16'd43000 + tries[15:0];
      toe_port  = 16'd80;
      peer_seq  = 32'h5000_0000 + tries * 32'd100;
      peer_ack  = 0;
      peer_win  = 16'hFFFF;
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, 0, F_SYN, 16'hFFFF, syn_options(1), no_bytes()));
      expect_flags(F_SYN | F_ACK, "SYN-ACK (ISS hunt)", p, 100000, 1);
      if (err_count) return;
      iss = f_seq(p);
      off = iss[17:0];
      peer_ack = iss + 1;
      peer_seq += 1;
      if (off > (BUF_SIZE - 5000) && off < (BUF_SIZE - 2000)) begin
        peer_send_seg(F_ACK, no_bytes());   // establish
        found = 1;
      end
      else begin
        peer_send_seg(F_RST, no_bytes());   // abort, free the session
        repeat (300) @(posedge ap_clk);
      end
      tries++;
    end
    tb_check(found, $sformatf("found ISS near buffer boundary after %0d tries", tries));
    if (!found) return;
    tb_info($sformatf("using ISS=%0d (offset %0d, %0d bytes below wrap)",
                      iss, off, BUF_SIZE - off));
    notif_q.delete();   // drop close notifications from aborted tries

    // learn session id: peer sends 8B
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 100000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;

    // app sends 4x1400B; payload stream crosses the buffer boundary
    data_base = iss + 1;
    sent.delete();
    for (int i = 0; i < 4; i++) begin
      pay = gen_payload(1400, 8'h50 + i[7:0]);
      app_send(sid, pay, rsp);
      tb_check(rsp.error == 0, $sformatf("app_send %0d accepted", i));
      foreach (pay[j]) sent.push_back(pay[j]);
    end
    // collect the 4 original data segments
    got = 0;
    while (got < 4) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout collecting data segments"); return; end
      if (f_payload_len(p) == 1400) got++;
    end

    // ACK first segment, then 3 dup ACKs -> fast retransmit replays from
    // ackd across the buffer wrap (DDR read gets split)
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, data_base + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (100) @(posedge ap_clk);
    for (int i = 0; i < 3; i++) begin
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, data_base + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));
      repeat (50) @(posedge ap_clk);
    end

    // verify every retransmitted byte against the original stream
    rt_bytes = 0;
    while (rt_bytes < 4200) begin
      bytes_t d;
      int soff;
      wait_pkt(p, 500000, ok);
      if (!ok) begin tb_error("timeout waiting for retransmissions"); return; end
      if (f_payload_len(p) == 0) continue;
      d = f_payload(p);
      soff = f_seq(p) - data_base;
      tb_check(soff >= 0 && soff + d.size() <= 5600, "rtx segment within sent range");
      for (int i = 0; i < d.size(); i++)
        if (d[i] !== sent[soff + i]) begin
          tb_check(0, $sformatf("rtx payload mismatch at stream offset %0d (%h != %h)",
                                soff + i, d[i], sent[soff + i]));
          return;
        end
      tb_info($sformatf("rtx segment seq-off=%0d len=%0d verified", soff, d.size()));
      rt_bytes += d.size();
    end

    // clean up: ACK everything
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, data_base + 5600, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
  endtask

  // F4 regression: the peer's MSS option from its SYN must cap the size of
  // retransmitted segments (session MSS = min(peer MSS, local MSS)).
  task automatic test_mss_option();
    bytes_t p, opts, pay;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] toe_iss, d0_seq;
    int got, rt_bytes;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");

    // SYN with MSS = 536
    opts = new[8];
    opts[0]=8'h02; opts[1]=8'h04; opts[2]=8'h02; opts[3]=8'h18;   // MSS 536
    opts[4]=8'h03; opts[5]=8'h03; opts[6]=8'h02; opts[7]=8'h01;   // WS 2 + NOP
    peer_port = 16'd41070; toe_port = 16'd80;
    peer_seq  = 32'h6000_0000; peer_win = 16'hFFFF;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, opts, no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 100000, 1);
    if (err_count) return;
    toe_iss  = f_seq(p);
    peer_ack = toe_iss + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());

    // learn session id
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // app sends 1000B; since P1 the initial transmission is also segmented
    // by the session MSS (536+464); peer never ACKs it
    pay = gen_payload(1000, 8'h90);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, "app_send accepted");
    got = 0;
    d0_seq = 0;
    while (got < 1000) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for data segment"); return; end
      if (f_payload_len(p) == 0) continue;
      if (got == 0) d0_seq = f_seq(p);
      tb_check(f_payload_len(p) <= 536,
               $sformatf("initial segment len=%0d respects peer MSS 536", f_payload_len(p)));
      if (f_payload_len(p) > 536) return;
      got += f_payload_len(p);
    end

    // 3 dup ACKs (ack = snd_una) -> fast retransmit, now chunked by MSS 536
    for (int i = 0; i < 3; i++) begin
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, d0_seq, F_ACK, peer_win, no_bytes(), no_bytes()));
      repeat (50) @(posedge ap_clk);
    end
    rt_bytes = 0;
    while (rt_bytes < 1000) begin
      bytes_t d;
      int soff;
      wait_pkt(p, 500000, ok);
      if (!ok) begin tb_error("timeout waiting for retransmissions"); return; end
      if (f_payload_len(p) == 0) continue;
      tb_check(f_payload_len(p) <= 536,
               $sformatf("rtx segment len=%0d respects peer MSS 536", f_payload_len(p)));
      if (f_payload_len(p) > 536) return;
      d = f_payload(p);
      soff = f_seq(p) - d0_seq;
      for (int i = 0; i < d.size(); i++)
        if (d[i] !== pay[soff + i]) begin
          tb_check(0, $sformatf("rtx payload mismatch at offset %0d", soff + i));
          return;
        end
      rt_bytes += d.size();
    end
    tb_check(rt_bytes == 1000, "all 1000 bytes retransmitted in MSS-sized chunks");

    // clean up
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_seq + 1000, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
  endtask

  // P1 regression: an app write larger than the session MSS must be sent as
  // multiple MSS-sized segments read back from the TX buffer (previously it
  // went out as one oversized segment via the bypass stream).
  task automatic test_tx_segmentation();
    bytes_t p, opts, pay;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] toe_iss, data_base;
    int got_bytes;
    bytes_t d;
    int soff;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");

    // SYN with MSS = 536
    opts = new[8];
    opts[0]=8'h02; opts[1]=8'h04; opts[2]=8'h02; opts[3]=8'h18;   // MSS 536
    opts[4]=8'h03; opts[5]=8'h03; opts[6]=8'h02; opts[7]=8'h01;   // WS 2 + NOP
    peer_port = 16'd41080; toe_port = 16'd80;
    peer_seq  = 32'h7000_0000; peer_win = 16'hFFFF;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, opts, no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 100000, 1);
    if (err_count) return;
    toe_iss  = f_seq(p);
    peer_ack = toe_iss + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());

    // learn session id
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // oversized app write: 1400B > session MSS 536 -> 536+536+328
    data_base = toe_iss + 1;
    pay = gen_payload(1400, 8'hB0);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, "app_send 1400B accepted");
    got_bytes = 0;
    while (got_bytes < 1400) begin
      wait_pkt(p, 500000, ok);
      if (!ok) begin tb_error("timeout collecting segmented data"); return; end
      if (f_payload_len(p) == 0) continue;
      tb_check(f_payload_len(p) <= 536,
               $sformatf("segment len=%0d respects session MSS", f_payload_len(p)));
      if (f_payload_len(p) > 536) return;
      d = f_payload(p);
      soff = f_seq(p) - data_base;
      tb_check(soff == got_bytes, $sformatf("segment seq offset %0d contiguous (expect %0d)", soff, got_bytes));
      for (int i = 0; i < d.size(); i++)
        if (d[i] !== pay[soff + i]) begin
          tb_check(0, $sformatf("segmented payload mismatch at offset %0d", soff + i));
          return;
        end
      got_bytes += d.size();
    end
    tb_check(got_bytes == 1400, "all 1400 bytes sent in MSS-sized segments");
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, data_base + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));

    // bypass fast path must still work right after (arbiter not desynced)
    pay = gen_payload(100, 8'hC5);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, "app_send 100B accepted");
    got_bytes = 0;
    while (!got_bytes) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for 100B segment"); return; end
      if (f_payload_len(p) == 0) continue;
      tb_check(f_payload_len(p) == 100, "single 100B bypass segment");
      d = f_payload(p);
      tb_check(f_seq(p) == data_base + 1400, "100B segment seq");
      for (int i = 0; i < d.size() && i < 100; i++)
        if (d[i] !== pay[i]) begin
          tb_check(0, $sformatf("bypass payload mismatch at %0d", i));
          return;
        end
      got_bytes = 1;
    end
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, data_base + 1500, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
  endtask

  // P3 regression: RFC 5681 fast recovery. The congestion-window trajectory
  // is verified by probing the tx_sar_table BRAMs:
  //   IW10 (14600) -> +MSS on new ACK (16060) -> 3rd dup ACK enters recovery
  //   (ssthresh = flight/2 = 4900, cwnd = ssthresh+3*MSS = 9280) -> 4th dup
  //   inflates (+MSS = 10740) -> recovery-exit ACK deflates to ssthresh (4900).
  task automatic test_fast_recovery();
    bytes_t p, pay;
    bit ok, ok2;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] d0_seq;
    int got;
    int unsigned cw, ssth;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41100);
    if (err_count) return;
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // IW10 = 14600, plus one slow-start increment from the handshake ACK
    cw = dut.tx_sar_table_U0.tx_table_cong_window_V_U.ram[sid];
    tb_check(cw == 14600 + 1460, $sformatf("initial cwnd=%0d expected 16060 (IW10 + handshake ACK)", cw));

    // 8 x 1400B in flight
    for (int i = 0; i < 8; i++) begin
      pay = gen_payload(1400, 8'h30 + i[7:0]);
      app_send(sid, pay, rsp);
      tb_check(rsp.error == 0, $sformatf("app_send %0d accepted", i));
    end
    got = 0;
    while (got < 8) begin
      wait_pkt(p, 400000, ok2);
      if (!ok2) begin tb_error("timeout collecting data segments"); return; end
      if (f_payload_len(p) == 1400) begin
        if (got == 0) d0_seq = f_seq(p);
        got++;
      end
    end

    // new ACK for the first segment: slow start grows cwnd by one MSS
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_seq + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (3000) @(posedge ap_clk);
    cw = dut.tx_sar_table_U0.tx_table_cong_window_V_U.ram[sid];
    tb_check(cw == 16060 + 1460, $sformatf("cwnd=%0d expected 17520 after slow-start +MSS", cw));

    // 3 duplicate ACKs -> fast recovery entry
    for (int i = 0; i < 3; i++) begin
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, d0_seq + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));
      repeat (100) @(posedge ap_clk);
    end
    repeat (3000) @(posedge ap_clk);
    ssth = dut.tx_sar_table_U0.tx_table_slowstart_threshold_V_U.ram[sid];
    cw   = dut.tx_sar_table_U0.tx_table_cong_window_V_U.ram[sid];
    tb_check(ssth == 4900, $sformatf("ssthresh=%0d expected 4900 (flight 9800 / 2)", ssth));
    tb_check(cw == 9280, $sformatf("cwnd=%0d expected 9280 (ssthresh + 3*MSS) on recovery entry", cw));

    // one more duplicate ACK -> window inflation
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_seq + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (3000) @(posedge ap_clk);
    cw = dut.tx_sar_table_U0.tx_table_cong_window_V_U.ram[sid];
    tb_check(cw == 10740, $sformatf("cwnd=%0d expected 10740 (inflation +MSS)", cw));

    // full ACK -> recovery exit, cwnd deflates to ssthresh
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_seq + 11200, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (3000) @(posedge ap_clk);
    cw = dut.tx_sar_table_U0.tx_table_cong_window_V_U.ram[sid];
    tb_check(cw == 4900, $sformatf("cwnd=%0d expected 4900 (deflated to ssthresh)", cw));

    while (net_pkts.size() > 0) void'(net_pkts.pop_front());
    repeat (2000) @(posedge ap_clk);
  endtask

  // N1 regression: half-close. After the peer's FIN the session enters
  // CLOSE_WAIT: the application can keep sending data and the TOE's own FIN
  // goes out only when the app closes. Also exercises FIN_WAIT_1->FIN_WAIT_2
  // (peer keeps sending data after ACKing our FIN).
  task automatic test_half_close();
    bytes_t p, pay;
    bit ok, ok2;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] fin_seq, data_seq;
    int base_deletes, nn;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");

    // --- part 1: peer half-closes, app keeps sending ---
    do_passive_handshake(16'd80, 16'd41130);
    if (err_count) return;
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    app_read(sid, 8, rdata); //drain so later reads start at fresh data
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    peer_send_seg(F_FIN | F_ACK, no_bytes());
    peer_seq += 1;
    wait_notif(n, 300000, ok);
    tb_check(ok && n.closed, "close notification (peer FIN)");
    // the ACK of the FIN must arrive with NO FIN from our side
    expect_flags(F_ACK, "ACK of peer FIN", p, 400000, 0);
    if (err_count) return;
    tb_check((f_flags(p) & F_FIN) == 0, "no FIN from TOE while in CLOSE_WAIT");
    tb_check(f_ack(p) == peer_seq, "FIN ACK number");

    // app can still send after the peer's FIN (this is the half-close)
    pay = gen_payload(500, 8'hE0);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, "app_send in CLOSE_WAIT accepted");
    ok = 0;
    while (!ok) begin
      wait_pkt(p, 400000, ok2);
      if (!ok2) begin tb_error("timeout waiting for CLOSE_WAIT data"); return; end
      if (f_payload_len(p) == 500) ok = 1;
    end
    tb_check(f_ack(p) == peer_seq, "CLOSE_WAIT data acks the peer FIN");
    data_seq = f_seq(p);
    begin
      bytes_t d;
      d = f_payload(p);
      for (int i = 0; i < 500 && i < d.size(); i++)
        if (d[i] !== pay[i]) begin
          tb_check(0, $sformatf("CLOSE_WAIT payload mismatch at %0d", i));
          return;
        end
    end
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, data_seq + 500, F_ACK, peer_win, no_bytes(), no_bytes()));

    // app closes -> FIN, peer ACKs -> LAST_ACK -> CLOSED (session released)
    base_deletes = cam_delete_count;
    app_close(sid);
    expect_flags(F_FIN, "FIN after app close", p, 400000, 1);
    if (err_count) return;
    tb_check(f_seq(p) == data_seq + 500, "FIN seq follows the CLOSE_WAIT data");
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, f_seq(p) + 1, F_ACK, peer_win, no_bytes(), no_bytes()));
    nn = 0;
    while (cam_delete_count == base_deletes && nn < 200000) begin
      @(posedge ap_clk); nn++;
    end
    tb_check(cam_delete_count > base_deletes, "session released after LAST_ACK");

    // --- part 2: we half-close first, peer keeps sending (FIN_WAIT_2) ---
    do_passive_handshake(16'd80, 16'd41131);
    if (err_count) return;
    peer_send_seg(F_ACK, gen_payload(8, 8'h02));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification (part 2)");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    app_read(sid, 8, rdata); //drain so the FIN_WAIT_2 read starts at fresh data
    expect_flags(F_ACK, "ACK of 8B (part 2)", p, 400000, 0);

    app_close(sid);
    expect_flags(F_FIN, "our FIN", p, 400000, 1);
    if (err_count) return;
    fin_seq = f_seq(p);
    // peer ACKs our FIN (no FIN of its own yet) -> FIN_WAIT_2
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, fin_seq + 1, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
    // peer keeps sending data; the TOE must accept and deliver it
    pay = gen_payload(300, 8'hF0);
    peer_ack = fin_seq + 1;
    peer_send_seg(F_ACK, pay);
    wait_notif(n, 300000, ok);
    tb_check(ok && n.length == 300, "data delivered in FIN_WAIT_2");
    if (!ok) return;
    peer_seq += 300;
    app_read(sid, 300, rdata);
    for (int i = 0; i < 300 && i < rdata.size(); i++)
      if (rdata[i] != pay[i]) begin
        tb_check(0, $sformatf("FIN_WAIT_2 payload mismatch at %0d", i));
        break;
      end
    // finally the peer closes too -> TIME_WAIT ACK
    peer_send_seg(F_FIN | F_ACK, no_bytes());
    peer_seq += 1;
    ok = 0;
    while (!ok) begin
      wait_pkt(p, 400000, ok2);
      if (!ok2) begin tb_error("timeout waiting for TIME_WAIT ACK"); return; end
      if ((f_flags(p) & F_ACK) && f_payload_len(p) == 0 && f_ack(p) == peer_seq) ok = 1;
    end
    tb_check(ok, "peer FIN acked (TIME_WAIT)");
    repeat (2000) @(posedge ap_clk);
  endtask

  // S2 regression: SACK blocks from the peer suppress retransmission of the
  // already-received range - selective retransmit instead of pure go-back-N.
  task automatic test_sack_selective_rtx();
    bytes_t p, pay;
    bit ok, ok2;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] d0;
    int got, rt_bytes, soff;
    bytes_t d;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");
    peer_port = 16'd41120; toe_port = 16'd80;
    peer_seq  = 32'h9000_0000; peer_win = 16'hFFFF;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, syn_options_sack(), no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 100000, 1);
    if (err_count) return;
    peer_ack = f_seq(p) + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // --- scenario A: first segment lost, peer SACKs [d0+1000, d0+4000) ---
    for (int i = 0; i < 4; i++) begin
      pay = gen_payload(1000, 8'h40 + i[7:0]);
      app_send(sid, pay, rsp);
      tb_check(rsp.error == 0, $sformatf("app_send A%0d accepted", i));
    end
    got = 0;
    while (got < 4) begin
      wait_pkt(p, 400000, ok2);
      if (!ok2) begin tb_error("timeout collecting A data"); return; end
      if (f_payload_len(p) == 1000) begin
        if (got == 0) d0 = f_seq(p);
        got++;
      end
    end
    for (int i = 0; i < 3; i++) begin
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, d0, F_ACK, peer_win,
                        sack_opt(d0 + 1000, d0 + 4000), no_bytes()));
      repeat (100) @(posedge ap_clk);
    end
    rt_bytes = 0;
    while (rt_bytes < 1000) begin
      wait_pkt(p, 500000, ok2);
      if (!ok2) begin tb_error("timeout waiting for selective rtx (A)"); return; end
      if (f_payload_len(p) == 0) continue;
      soff = f_seq(p) - d0;
      tb_check(soff >= 0 && soff + f_payload_len(p) <= 1000,
               $sformatf("rtx seq-off=%0d len=%0d stays outside the SACKed range",
                         soff, f_payload_len(p)));
      if (soff < 0 || soff + f_payload_len(p) > 1000) return;
      rt_bytes += f_payload_len(p);
    end
    tb_check(rt_bytes == 1000, "exactly the missing 1000B retransmitted (A)");
    // no further data may follow (the SACKed 3000B must not be resent)
    got = 0;
    while (!got) begin
      wait_pkt(p, 200000, ok2);
      if (!ok2) got = 1;
      else if (f_payload_len(p) != 0) begin
        tb_check(0, $sformatf("unexpected rtx of SACKed data at seq-off=%0d", f_seq(p) - d0));
        return;
      end
    end
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0 + 4000, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (3000) @(posedge ap_clk);

    // --- scenario B: middle hole, peer SACKs [b+1000, b+3000) ---
    // fresh connection: scenario A's recovery correctly deflated cwnd to
    // ssthresh (2000), which would reject 4 back-to-back 1000B writes
    begin
      bit [31:0] b;
      peer_port = 16'd41121;
      peer_seq  = 32'h9100_0000; peer_win = 16'hFFFF;
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, 0, F_SYN, 16'hFFFF, syn_options_sack(), no_bytes()));
      expect_flags(F_SYN | F_ACK, "SYN-ACK (B)", p, 100000, 1);
      if (err_count) return;
      peer_ack = f_seq(p) + 1;
      peer_seq += 1;
      peer_send_seg(F_ACK, no_bytes());
      peer_send_seg(F_ACK, gen_payload(8, 8'h02));
      wait_notif(n, 200000, ok);
      tb_check(ok, "data notification (B)");
      if (!ok) return;
      sid = n.sessionID;
      peer_seq += 8;
      expect_flags(F_ACK, "ACK of 8B (B)", p, 400000, 0);
      for (int i = 0; i < 4; i++) begin
        pay = gen_payload(1000, 8'h80 + i[7:0]);
        app_send(sid, pay, rsp);
        tb_check(rsp.error == 0, $sformatf("app_send B%0d accepted", i));
      end
      got = 0;
      while (got < 4) begin
        wait_pkt(p, 400000, ok2);
        if (!ok2) begin tb_error("timeout collecting B data"); return; end
        if (f_payload_len(p) == 1000) begin
          if (got == 0) b = f_seq(p);
          got++;
        end
      end
      for (int i = 0; i < 3; i++) begin
        net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                          peer_seq, b, F_ACK, peer_win,
                          sack_opt(b + 1000, b + 3000), no_bytes()));
        repeat (100) @(posedge ap_clk);
      end
      rt_bytes = 0;
      while (rt_bytes < 2000) begin
        wait_pkt(p, 500000, ok2);
        if (!ok2) begin tb_error("timeout waiting for selective rtx (B)"); return; end
        if (f_payload_len(p) == 0) continue;
        soff = f_seq(p) - b;
        tb_check((soff >= 0 && soff + f_payload_len(p) <= 1000) ||
                 (soff >= 3000 && soff + f_payload_len(p) <= 4000),
                 $sformatf("rtx seq-off=%0d len=%0d avoids the SACKed hole",
                           soff, f_payload_len(p)));
        if (!((soff >= 0 && soff + f_payload_len(p) <= 1000) ||
              (soff >= 3000 && soff + f_payload_len(p) <= 4000))) return;
        rt_bytes += f_payload_len(p);
      end
      tb_check(rt_bytes == 2000, "both unSACKed regions retransmitted (B)");
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, b + 4000, F_ACK, peer_win, no_bytes(), no_bytes()));
    end
    repeat (2000) @(posedge ap_clk);
  endtask

  // F3 regression: a FIN carrying no ACK flag must still close the
  // connection (it previously fell into the ignored default flag case).
  task automatic test_pure_fin_close();
    bytes_t p;
    bit ok;
    notif_t n;
    bit [15:0] sid;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");
    do_passive_handshake(16'd80, 16'd41060);
    if (err_count) return;

    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 8, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // pure FIN (ACK flag clear)
    peer_send_seg(F_FIN, no_bytes());
    peer_seq += 1;
    wait_notif(n, 300000, ok);
    tb_check(ok && n.closed, "close notification for pure FIN");
    // half-close: ACK only, FIN comes after the app closes
    expect_flags(F_ACK, "ACK of pure FIN", p, 400000, 0);
    if (err_count) return;
    tb_check((f_flags(p) & F_FIN) == 0, "no premature FIN (CLOSE_WAIT)");
    tb_check(f_ack(p) == peer_seq, "FIN ACK number");
    app_close(sid);
    expect_flags(F_FIN, "FIN after app close", p, 400000, 1);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq, "TOE FIN acks the peer FIN");
    peer_ack = f_seq(p) + 1;
    peer_send_seg(F_ACK, no_bytes());
    repeat (2000) @(posedge ap_clk);
  endtask

  // F1 regression: after the application drains the receive buffer, the TOE
  // must send a window-update ACK on its own (previously the peer could only
  // recover from a shrunken/zero window via its persist timer - issue #21).
  task automatic test_window_update();
    bytes_t p, pay;
    bit ok, got_ack;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    bit [15:0] w_before, w_after;
    int nn;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");
    do_passive_handshake(16'd80, 16'd41040);
    if (err_count) return;

    // peer sends 3 x 1400B; app does NOT read yet
    for (int i = 0; i < 3; i++) begin
      pay = gen_payload(1400, 8'h60 + i[7:0]);
      peer_send_seg(F_ACK, pay);
      wait_notif(n, 200000, ok);
      tb_check(ok && n.length == 1400, $sformatf("segment %0d notified", i));
      if (!ok) return;
      peer_seq += 1400;
    end
    sid = n.sessionID;

    // collect ACK covering all 4200B and note the shrunken window
    got_ack = 0; w_before = 16'hFFFF;
    while (!got_ack) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for ACK of 4200B"); return; end
      if ((f_flags(p) & F_ACK) && f_ack(p) == peer_seq) begin
        w_before = f_win(p);
        got_ack = 1;
      end
    end
    tb_check(w_before < 16'hFFFF, $sformatf("window shrunk to %0d while data unread", w_before));

    // drain any remaining queued packets
    while (net_pkts.size() > 0) void'(net_pkts.pop_front());

    // app reads everything (one read per notification - required in the
    // RX_DDR_BYPASS build where data is delivered per packet boundary)
    // -> expect a spontaneous window-update ACK
    for (int i = 0; i < 3; i++) begin
      app_read(sid, 1400, rdata);
      tb_check(rdata.size() == 1400, $sformatf("app_read %0d got 1400 bytes", i));
    end
    got_ack = 0;
    nn = 0;
    while (!got_ack && nn < 3) begin
      wait_pkt(p, 400000, ok);
      if (!ok) break;
      if ((f_flags(p) & F_ACK) && f_payload_len(p) == 0 && f_win(p) > w_before) begin
        w_after = f_win(p);
        got_ack = 1;
      end
      nn++;
    end
    tb_check(got_ack, "window-update ACK sent after app drained the buffer");
    if (got_ack)
      tb_info($sformatf("window update: %0d -> %0d", w_before, w_after));
    repeat (2000) @(posedge ap_clk);
  endtask

`ifdef NO_RX_BYPASS
  // S1 regression (RFC 2018): SACK-Permitted negotiation on the handshake and
  // one SACK block on duplicate ACKs while the single receive gap is open.
  task automatic test_sack_rx_gen();
    bytes_t p, opts, stream_data, segA, segB, segC;
    bit ok, found;
    byte unsigned val;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    bit [31:0] base_seq, sstart, send_;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");

    // --- connection WITH SACK-Permitted ---
    peer_port = 16'd41110; toe_port = 16'd80;
    peer_seq  = 32'h8000_0000; peer_win = 16'hFFFF;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, syn_options_sack(), no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 100000, 1);
    if (err_count) return;
    found = find_option(p, 8'h04, val);
    tb_check(found, "SYN-ACK echoes SACK-Permitted");
    found = find_option(p, 8'h03, val);
    tb_check(found, "SYN-ACK still carries WS");
    peer_ack = f_seq(p) + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());
    base_seq = peer_seq;

    stream_data = gen_payload(2000, 8'hD0);
    segA = new[1000]; for (int i = 0; i < 1000; i++) segA[i] = stream_data[i];
    segB = new[500];  for (int i = 0; i < 500;  i++) segB[i] = stream_data[1000+i];
    segC = new[500];  for (int i = 0; i < 500;  i++) segC[i] = stream_data[1500+i];

    // OOO segment B opens a gap -> dup ACK with SACK block [B, B+500)
    peer_seq = base_seq + 1000;
    peer_send_seg(F_ACK, segB);
    expect_flags(F_ACK, "dup ACK (gap open)", p, 200000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == base_seq, "dup ACK number");
    found = get_sack_block(p, sstart, send_);
    tb_check(found, "dup ACK carries a SACK block");
    if (found) begin
      tb_check(sstart == base_seq + 1000 && send_ == base_seq + 1500,
               $sformatf("SACK block [%0d,%0d) expected [%0d,%0d)",
                         sstart, send_, base_seq+1000, base_seq+1500));
    end

    // contiguous OOO segment C extends the buffered range -> block grows
    peer_seq = base_seq + 1500;
    peer_send_seg(F_ACK, segC);
    expect_flags(F_ACK, "dup ACK (gap extended)", p, 200000, 0);
    if (err_count) return;
    found = get_sack_block(p, sstart, send_);
    tb_check(found, "second dup ACK carries a SACK block");
    if (found) begin
      tb_check(sstart == base_seq + 1000 && send_ == base_seq + 2000,
               $sformatf("SACK block [%0d,%0d) expected [%0d,%0d)",
                         sstart, send_, base_seq+1000, base_seq+2000));
    end

    // fill the gap -> combined notification, subsequent ACKs carry no SACK
    peer_seq = base_seq;
    peer_send_seg(F_ACK, segA);
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 2000, "combined 2000B notified after gap fill");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq = base_seq + 2000;
    expect_flags(F_ACK, "ACK after gap fill", p, 400000, 0);
    if (err_count) return;
    found = get_sack_block(p, sstart, send_);
    tb_check(!found, "no SACK block once the gap is closed");
    app_read(sid, 2000, rdata);
    tb_check(rdata.size() == 2000, "read 2000 bytes");
    for (int i = 0; i < 2000 && i < rdata.size(); i++)
      if (rdata[i] != stream_data[i]) begin
        tb_check(0, $sformatf("data mismatch at byte %0d", i));
        break;
      end

    // --- connection WITHOUT SACK-Permitted: no SACK blocks may appear ---
    do_passive_handshake(16'd80, 16'd41111);
    if (err_count) return;
    base_seq = peer_seq;
    peer_seq = base_seq + 1000;
    peer_send_seg(F_ACK, segB);
    expect_flags(F_ACK, "dup ACK (no-SACK session)", p, 200000, 1);
    if (err_count) return;
    found = get_sack_block(p, sstart, send_);
    tb_check(!found, "no SACK block on a session without SACK-Permitted");
    // clean up the gap
    peer_seq = base_seq;
    peer_send_seg(F_ACK, gen_payload(1000, 8'h11));
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 1500, "no-SACK session still reassembles");
    repeat (2000) @(posedge ap_clk);
  endtask

  // F2 regression: (A) a segment far beyond the advertised window used to be
  // accepted through a WINDOW_BITS-truncated check and aliased onto valid
  // buffer content, corrupting unread data; (B) OOO segments whose sequence
  // numbers cross the 2^32 wrap were mis-dropped by plain 32-bit compares.
  task automatic test_seq_window_check();
    bytes_t p, payA, stream_data, segA, segB;
    bit ok;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    bit [31:0] base_seq;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");

    // --- subtest A: far-future segment must not alias onto buffered data ---
    do_passive_handshake(16'd80, 16'd41050);
    if (err_count) return;
    base_seq = peer_seq;
    payA = gen_payload(1000, 8'hA5);
    peer_send_seg(F_ACK, payA);
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 1000, "in-order segment notified");
    if (!ok) return;
    sid = n.sessionID;
    // segment one full buffer size ahead - same physical offset as payA
    peer_seq = base_seq + BUF_SIZE;
    peer_send_seg(F_ACK, gen_payload(500, 8'hEE));
    expect_flags(F_ACK, "dup ACK for out-of-window segment", p, 200000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == base_seq + 1000, "ACK covers only in-order data");
    peer_seq = base_seq + 1000;
    // unread data must be intact (pre-fix: first 500B overwritten with 0xEE..)
    app_read(sid, 1000, rdata);
    for (int i = 0; i < 1000 && i < rdata.size(); i++)
      if (rdata[i] != payA[i]) begin
        tb_check(0, $sformatf("buffered data corrupted at byte %0d (%h != %h)",
                              i, rdata[i], payA[i]));
        break;
      end
    tb_info("subtest A done: no aliasing of out-of-window segment");

    // --- subtest B: OOO reassembly across the 2^32 sequence wrap ---
    do_passive_handshake(16'd80, 16'd41051, 32'hFFFFFE00);
    if (err_count) return;
    base_seq = peer_seq;   // 0xFFFFFE01
    stream_data = gen_payload(1500, 8'h5A);
    segA = new[1000]; for (int i = 0; i < 1000; i++) segA[i] = stream_data[i];
    segB = new[500];  for (int i = 0; i < 500;  i++) segB[i] = stream_data[1000+i];
    peer_seq = base_seq + 1000;   // wraps past 2^32
    peer_send_seg(F_ACK, segB);
    expect_flags(F_ACK, "dup ACK (wrap OOO gap open)", p, 200000, 1);
    if (err_count) return;
    tb_check(f_ack(p) == base_seq, "gap-open ACK number");
    tb_check(notif_q.size() == 0, "no notification for wrapped OOO segment");
    peer_seq = base_seq;
    peer_send_seg(F_ACK, segA);
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 1500, "combined 1500B notified across seq wrap");
    if (!ok) return;
    peer_seq = base_seq + 1500;
    app_read(n.sessionID, 1500, rdata);
    tb_check(rdata.size() == 1500, "read 1500 bytes");
    for (int i = 0; i < 1500 && i < rdata.size(); i++)
      if (rdata[i] != stream_data[i]) begin
        tb_check(0, $sformatf("wrap reassembly mismatch at byte %0d", i));
        break;
      end
    repeat (2000) @(posedge ap_clk);
  endtask

  // B6 regression: with a gap open, a retransmitted segment that starts at
  // recvd but OVERLAPS the buffered premature data (seq+len > offset) hit a
  // branchless path in rx_engine - no SAR update, no drop token - wedging
  // rxPackageDropper and the whole RX data path permanently.
  task automatic test_rx_ooo_overlap();
    bytes_t p, stream_data, segA, segB, segC;
    bit ok;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    bit [31:0] base_seq;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");
    do_passive_handshake(16'd80, 16'd41020);
    if (err_count) return;
    base_seq = peer_seq;

    // consistent byte stream: A' = S[0:1200), B = S[1000:1500)
    stream_data = gen_payload(1500, 8'h70);
    segA = new[1200]; for (int i = 0; i < 1200; i++) segA[i] = stream_data[i];
    segB = new[500];  for (int i = 0; i < 500;  i++) segB[i] = stream_data[1000+i];

    // open a gap: B at base+1000
    peer_seq = base_seq + 1000;
    peer_send_seg(F_ACK, segB);
    expect_flags(F_ACK, "dup ACK (gap open)", p, 100000, 0);
    if (err_count) return;

    // overlapping retransmission: A' at base, len 1200 (> offset by 200)
    peer_seq = base_seq;
    peer_send_seg(F_ACK, segA);
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 1500, $sformatf("overlap fill notified len=%0d expected 1500",
                                               ok ? n.length : 16'd0));
    if (!ok) return;
    sid = n.sessionID;
    peer_seq = base_seq + 1500;

    app_read(sid, 1500, rdata);
    tb_check(rdata.size() == 1500, "read 1500 bytes");
    for (int i = 0; i < 1500 && i < rdata.size(); i++)
      if (rdata[i] != stream_data[i]) begin
        tb_check(0, $sformatf("stream mismatch at byte %0d (%h != %h)",
                              i, rdata[i], stream_data[i]));
        break;
      end

    // RX path must still be alive: one more in-order segment
    segC = gen_payload(100, 8'hC0);
    peer_send_seg(F_ACK, segC);
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 100, "rx path alive after overlap handling");
    repeat (2000) @(posedge ap_clk);
  endtask
`endif

  // B5 regression (issues #42/#43): the SYN option parser must survive
  // malformed / exotic option lists and clamp window-scale shifts > 14.
  // Pre-fix RTL deadlocks the whole RX path on a zero-length unknown option.
  task automatic test_opt_parser_robust();
    bytes_t p, opts;
    bit ok, found;
    byte unsigned val;
    notif_t n;
    bit ws_expected;

    app_listen(16'd80, ok);
    tb_check(ok, "listen(80)");

    // --- conn 1: SYN with malformed option (kind 254, length 0) ---
    // pre-fix: option parser spins forever -> no SYN-ACK ever
    opts = new[4];
    opts[0] = 8'hFE; opts[1] = 8'h00; opts[2] = 8'h00; opts[3] = 8'h00;
    peer_port = 16'd41010; toe_port = 16'd80;
    peer_seq  = 32'h4000_0000; peer_win = 16'hFFFF;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, opts, no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK for malformed-option SYN", p, 100000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq + 1, "SYN-ACK ack number (conn 1)");
    found = find_option(p, 8'h03, val);
    tb_check(!found, "no WS option echoed when peer WS was unparseable");
    peer_ack = f_seq(p) + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());
    // session must be functional: send 8B, expect notification
    peer_send_seg(F_ACK, gen_payload(8, 8'h11));
    wait_notif(n, 100000, ok);
    tb_check(ok && n.length == 8, "conn 1 delivers data after malformed options");
    peer_seq += 8;

    // --- conn 2: exotic but legal list: MSS, unknown kind 253 len 4, NOP, WS=14 ---
    opts = new[12];
    opts[0]=8'h02; opts[1]=8'h04; opts[2]=8'h05; opts[3]=8'hB4;   // MSS 1460
    opts[4]=8'hFD; opts[5]=8'h04; opts[6]=8'hAA; opts[7]=8'hBB;   // unknown, len 4
    opts[8]=8'h01;                                                // NOP
    opts[9]=8'h03; opts[10]=8'h03; opts[11]=8'h0E;                // WS shift=14
    peer_port = 16'd41011;
    peer_seq  = 32'h4100_0000;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, opts, no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK for exotic-option SYN", p, 100000, 0);
    if (err_count) return;
    found = find_option(p, 8'h03, val);
    tb_check(found, "WS option present after skipping unknown option");
    peer_ack = f_seq(p) + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());

    // --- conn 3: out-of-range WS shift 15 (must be clamped, not corrupt state) ---
    opts = new[8];
    opts[0]=8'h02; opts[1]=8'h04; opts[2]=8'h05; opts[3]=8'hB4;   // MSS 1460
    opts[4]=8'h03; opts[5]=8'h03; opts[6]=8'h0F; opts[7]=8'h01;   // WS shift=15 + NOP
    peer_port = 16'd41012;
    peer_seq  = 32'h4200_0000;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, opts, no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK for WS=15 SYN", p, 100000, 0);
    if (err_count) return;
    found = find_option(p, 8'h03, val);
    tb_check(found, "WS option echoed for WS=15 SYN");
    peer_ack = f_seq(p) + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());
    peer_send_seg(F_ACK, gen_payload(16, 8'h22));
    wait_notif(n, 100000, ok);
    tb_check(ok && n.length == 16, "conn 3 delivers data with clamped WS");
    repeat (2000) @(posedge ap_clk);
  endtask

  // B1 regression: an actively-opened session's ephemeral port must be
  // returned to free_port_table when the session is torn down (peer RST ->
  // CLOSED -> releaseSession -> port release). Uses a hierarchical probe into
  // the freePortTable BRAM. Fails on RTL built before the endianness fix in
  // session_lookup_controller.cpp (reverseLookupTableInterface).
  task automatic test_port_release();
    bytes_t p;
    bit ok;
    notif_t n;
    int nn;
    bit [15:0] eph_port;
    int unsigned free_idx;
    bit [31:0] toe_iss;

    app_open(PEER_IP, 16'd60000);
    expect_flags(F_SYN, "SYN", p, 100000, 0);
    if (err_count) return;
    eph_port = f_src_port(p);
    tb_check(eph_port >= 16'd32768, $sformatf("ephemeral port %0d in 32768..65535", eph_port));
    free_idx = eph_port & 16'h7FFF;
    tb_check(dut.free_port_table_U0.freePortTable_U.ram[free_idx] === 1'b1,
             "freePortTable entry marked used after allocation");

    toe_iss   = f_seq(p);
    toe_port  = eph_port;
    peer_port = 16'd60000;
    peer_seq  = 32'h3000_0000;
    peer_ack  = toe_iss + 1;
    peer_win  = 16'hFFFF;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, peer_ack, F_SYN | F_ACK, 16'hFFFF,
                      syn_options(1), no_bytes()));
    peer_seq += 1;
    expect_flags(F_ACK, "final ACK of 3WHS", p, 100000, 0);
    if (err_count) return;
    nn = 0;
    while (openrsp_q.size() == 0 && nn < 50000) begin @(posedge ap_clk); nn++; end
    tb_check(openrsp_q.size() > 0, "openConnRsp received");
    if (openrsp_q.size()) void'(openrsp_q.pop_front());

    // peer resets the connection -> immediate teardown (no TIME_WAIT)
    peer_send_seg(F_RST, no_bytes());
    wait_notif(n, 300000, ok);
    tb_check(ok && n.closed, "app notified of RST close");

    nn = 0;
    while (dut.free_port_table_U0.freePortTable_U.ram[free_idx] !== 1'b0 && nn < 200000) begin
      @(posedge ap_clk); nn++;
    end
    tb_check(dut.free_port_table_U0.freePortTable_U.ram[free_idx] === 1'b0,
             $sformatf("ephemeral port %0d released in freePortTable", eph_port));
    nn = 0;
    while (cam_delete_count == 0 && nn < 50000) begin
      @(posedge ap_clk); nn++;
    end
    tb_check(cam_delete_count > 0, "CAM DELETE issued on session teardown");
    repeat (2000) @(posedge ap_clk);
  endtask

  // P2 regression. Requires a timer-scaled RTL build. After a few RTT
  // samples the RTO must shrink from the initial 1s to ~RTO_MIN (250ms
  // scaled), and consecutive RTOs must back off exponentially (RFC 6298).
  task automatic test_adaptive_rto();
    bytes_t p, pay;
    bit ok, ok2;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    realtime t_data, t_rtx1, t_rtx2;
    real d1, d2;
    bit [31:0] seqd;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41090);
    if (err_count) return;
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // three prompt data/ACK exchanges -> RTT samples -> RTO converges to RTO_MIN
    for (int k = 0; k < 3; k++) begin
      pay = gen_payload(100, 8'h20 + k[7:0]);
      app_send(sid, pay, rsp);
      tb_check(rsp.error == 0, $sformatf("app_send sample %0d accepted", k));
      ok = 0;
      while (!ok) begin
        wait_pkt(p, 400000, ok2);
        if (!ok2) begin tb_error("timeout waiting for sample data segment"); return; end
        if (f_payload_len(p) == 100) ok = 1;
      end
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, f_seq(p) + 100, F_ACK, peer_win, no_bytes(), no_bytes()));
      repeat (3000) @(posedge ap_clk); // let the estimator update the RTO
    end

    // send data and stay silent: measure the RTO retransmission spacing
    pay = gen_payload(200, 8'h55);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, "app_send probe accepted");
    ok = 0;
    while (!ok) begin
      wait_pkt(p, 400000, ok2);
      if (!ok2) begin tb_error("timeout waiting for original segment"); return; end
      if (f_payload_len(p) == 200) begin t_data = $realtime; seqd = f_seq(p); ok = 1; end
    end
    ok = 0;
    while (!ok) begin
      wait_pkt(p, 2000000, ok2);
      if (!ok2) begin tb_error("timeout waiting for 1st RTO retransmission"); return; end
      if (f_payload_len(p) == 200 && f_seq(p) == seqd) begin t_rtx1 = $realtime; ok = 1; end
    end
    ok = 0;
    while (!ok) begin
      wait_pkt(p, 2000000, ok2);
      if (!ok2) begin tb_error("timeout waiting for 2nd RTO retransmission"); return; end
      if (f_payload_len(p) == 200 && f_seq(p) == seqd) begin t_rtx2 = $realtime; ok = 1; end
    end
    d1 = t_rtx1 - t_data;
    d2 = t_rtx2 - t_rtx1;
    // scaled RTO_MIN (250ms/1000) ~= 40 ticks -> ~256us; pre-P2 fixed 1s/1000 ~= 1ms
    tb_info($sformatf("RTO gaps: first=%.0fns second=%.0fns", d1, d2));
    tb_check(d1 > 100000 && d1 < 700000,
             $sformatf("first RTO %.0fns ~ RTO_MIN (adaptive), not the fixed 1s", d1));
    tb_check(d2 > d1 * 1.4 && d2 < d1 * 3.0,
             $sformatf("second RTO %.0fns ~ 2x first (exponential backoff)", d2));
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, seqd + 200, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
  endtask

  // B7 regression. Requires a timer-scaled RTL build. After the TOE sends a
  // FIN and the peer stays silent, the RTO must retransmit the FIN itself -
  // pre-fix the RT path treated the FIN's phantom sequence byte as 1 byte of
  // data and emitted a bogus 1-byte data segment at the FIN's seq number.
  task automatic test_fin_retransmit();
    bytes_t p;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    bit [31:0] fin_seq;
    int got;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41030);
    if (err_count) return;

    // learn session id
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 100000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // app closes; peer neither ACKs the FIN nor responds
    app_close(sid);
    expect_flags(F_FIN, "original FIN", p, 300000, 1);
    if (err_count) return;
    fin_seq = f_seq(p);
    tb_check(f_payload_len(p) == 0, "FIN carries no payload");

    // wait for the retransmission (scaled RTO1 ~157k cycles); any data
    // segment before it is the phantom-byte bug
    got = 0;
    while (!got) begin
      wait_pkt(p, 1000000, ok);
      if (!ok) begin tb_error("timeout waiting for FIN retransmission"); return; end
      if (f_payload_len(p) != 0) begin
        tb_check(0, $sformatf("bogus %0d-byte data segment at seq=%0d before FIN rtx",
                              f_payload_len(p), f_seq(p)));
        return;
      end
      if (f_flags(p) & F_FIN) begin
        tb_check(f_seq(p) == fin_seq, $sformatf("FIN rtx seq=%0d expected %0d", f_seq(p), fin_seq));
        got = 1;
      end
      // pure ACKs are ignored
    end

    // ACK the FIN to clean up
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, fin_seq + 1, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
  endtask

  // Requires the timer-scaled RTL build (buildsim, TCP_STACK_SIM_TIMER_SCALE=1000):
  // first RTO = "1s"/1000 ~= 157k cycles. With the normal build this test
  // would need ~156M cycles and will just time out.
  task automatic test_rto_retransmit();
    bytes_t p, pay;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] d_seq;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41005);
    if (err_count) return;

    // learn session id
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 100000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    // consume delayed ACK of the 8B
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // app sends 500B; peer does NOT acknowledge
    pay = gen_payload(500, 8'h44);
    app_send(sid, pay, rsp);
    tb_check(rsp.error == 0, "app_send accepted");
    expect_flags(F_ACK, "original data segment", p, 300000, 0);
    if (err_count) return;
    tb_check(f_payload_len(p) == 500, "original segment length");
    d_seq = f_seq(p);

    // wait for RTO retransmission (scaled RTO1 ~157k cycles)
    expect_flags(F_ACK, "RTO retransmission", p, 1000000, 0);
    if (err_count) return;
    tb_check(f_payload_len(p) == 500, "retransmitted segment length");
    tb_check(f_seq(p) == d_seq, $sformatf("rtx seq=%0d expected %0d", f_seq(p), d_seq));

    // now ACK it - connection should recover
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d_seq + 500, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
  endtask

  // N2: TCP keepalive (RFC 1122 4.2.3.6). Requires the timer-scaled RTL build
  // (buildsim, TCP_STACK_SIM_TIMER_SCALE=1000): scaled KEEPALIVE_IDLE =
  // "60s"/1000 -> 9376 ticks ~ 9.38M cycles, probes every "10s"/1000 ~ 1.56M
  // cycles, 4 unanswered probes abort the session. Run with the 40M-cycle
  // watchdog (selected automatically by testname).
  task automatic test_keepalive();
    bytes_t p;
    bit ok, got;
    notif_t n;
    bit [15:0] sid;
    bit [31:0] snd_nxt;
    bit [35:0] ka_raw;
    bit [31:0] ka_time;
    realtime t_prev, t_now;
    real d;
    int nn;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41300);
    if (err_count) return;

    // learn session id
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 100000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);
    if (err_count) return;
    snd_nxt = f_seq(p);

    // (A) whitebox: the idle counter counts down while the session is quiet
    // and is reloaded by any received in-window segment
    repeat (1_500_000) @(posedge ap_clk);
    ka_raw = dut.keepalive_timer_U0.keepaliveTimerTable_U.ram[sid];
    ka_time = ka_raw[31:0];
    tb_check(ka_raw[35], "keepalive entry active");
    tb_check(ka_time > 32'd7000 && ka_time < 32'd8500,
             $sformatf("idle counter ran down (time=%0d)", ka_time));
    peer_send_seg(F_ACK, gen_payload(8, 8'h02));
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 2nd 8B", p, 400000, 0);
    wait_notif(n, 100000, ok);   // consume the data notification of the 2nd 8B
    tb_check(ok, "2nd data notification");
    repeat (5000) @(posedge ap_clk);
    ka_raw = dut.keepalive_timer_U0.keepaliveTimerTable_U.ram[sid];
    ka_time = ka_raw[31:0];
    tb_check(ka_time > 32'd9000,
             $sformatf("idle counter reloaded by rx activity (time=%0d)", ka_time));
    if (err_count) return;

    // (B) total silence -> 4 probes: pure ACKs with seq = snd_nxt - 1
    for (int k = 1; k <= 4; k++) begin
      got = 0;
      while (!got) begin
        wait_pkt(p, (k == 1) ? 11_000_000 : 2_500_000, ok);
        if (!ok) begin
          tb_error($sformatf("timeout waiting for keepalive probe %0d", k));
          return;
        end
        // skip stragglers (e.g. a delayed ACK at seq == snd_nxt)
        if (f_payload_len(p) == 0 && f_seq(p) == snd_nxt - 1) got = 1;
      end
      t_now = $realtime;
      tb_check((f_flags(p) & F_ACK) != 0, $sformatf("probe %0d is an ACK", k));
      tb_check(f_ack(p) == peer_seq, $sformatf("probe %0d ack field", k));
      if (k > 1) begin
        d = t_now - t_prev;
        // scaled KEEPALIVE_INTERVAL = 1564 ticks ~1.56M cycles ~= 10ms
        // ($realtime is in ns: timescale 1ns/1ps)
        tb_check(d > 7_000_000.0 && d < 14_000_000.0,
                 $sformatf("probe %0d spacing %.0fns ~ 10ms(scaled)", k, d));
      end
      t_prev = t_now;
      tb_info($sformatf("keepalive probe %0d seq=%0d", k, f_seq(p)));
      // whitebox: probe counter tracks the unanswered probes
      ka_raw = dut.keepalive_timer_U0.keepaliveTimerTable_U.ram[sid];
      tb_check(ka_raw[34:32] == k[2:0], $sformatf("probe counter == %0d", k));
    end

    // (C) after 4 unanswered probes the session is aborted
    wait_notif(n, 3_000_000, ok);
    tb_check(ok, "close notification after unanswered probes");
    if (ok) begin
      tb_check(n.sessionID == sid, "notification session id");
      tb_check(n.closed == 1, "notification marks session closed");
    end
    nn = 0;
    while (cam_delete_count == 0 && nn < 200_000) begin @(posedge ap_clk); nn++; end
    tb_check(cam_delete_count > 0, "CAM DELETE issued on keepalive abort");
    // and probing stops
    wait_pkt(p, 2_000_000, ok);
    tb_check(!ok, "no further probes after abort");
  endtask

  // T1: RFC 7323 timestamps negotiation, echo and payload realignment.
  // Verifies: TS on SYN-ACK with correct TSecr, TS on pure ACKs and data
  // segments (dataOffset 8), TSecr always echoes the newest peer TSval, and
  // that payload behind the 12B option is intact (the TX body is shifted by
  // 4 bytes in the header builder) for aligned and unaligned lengths.
  task automatic test_ts_negotiate();
    bytes_t p, pay;
    bit ok, got;
    notif_t n;
    bit [15:0] sid;
    bit [31:0] toe_ts, tv, te;
    bit [31:0] peer_tsclock;
    txrsp_t rsp;
    int lens[3];

    app_listen(16'd80, ok);
    peer_tsclock = 32'd10_000;
    do_ts_handshake(16'd80, 16'd41400, 32'h3000_0000, peer_tsclock, toe_ts);
    if (err_count) return;

    // 8B data with a fresh timestamp -> delivered, ACK echoes the new TSval
    peer_tsclock += 10;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, peer_ack, F_ACK, peer_win,
                      ts_opt(peer_tsclock, toe_ts), gen_payload(8, 8'h11)));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq, "ACK number");
    ok = get_ts_option(p, tv, te);
    tb_check(ok, "ACK carries timestamps");
    if (ok) tb_check(te == peer_tsclock, $sformatf("ACK TSecr=%0d expected %0d", te, peer_tsclock));
    toe_ts = tv;

    // TOE data segments: 100B (unaligned tail), 4B (fits the merged word)
    // and 8B; each must arrive with TS option and undamaged payload
    lens[0] = 100; lens[1] = 4; lens[2] = 8;
    for (int k = 0; k < 3; k++) begin
      pay = gen_payload(lens[k], 8'h40 + k[7:0]);
      app_send(sid, pay, rsp);
      tb_check(rsp.error == 0, $sformatf("app_send %0dB accepted", lens[k]));
      got = 0;
      while (!got) begin
        wait_pkt(p, 400000, ok);
        if (!ok) begin tb_error($sformatf("timeout waiting for %0dB data", lens[k])); return; end
        if (f_payload_len(p) == lens[k]) got = 1;
      end
      ok = get_ts_option(p, tv, te);
      tb_check(ok, $sformatf("%0dB data segment carries timestamps", lens[k]));
      if (ok) tb_check(te == peer_tsclock, $sformatf("data TSecr=%0d expected %0d", te, peer_tsclock));
      tb_check(f_options(p).size() == 12, "data segment option length 12B");
      begin
        bytes_t rx = f_payload(p);
        bit match = 1;
        foreach (pay[i]) if (rx[i] !== pay[i]) match = 0;
        tb_check(match, $sformatf("%0dB payload intact behind TS option", lens[k]));
      end
      // ACK the data with a fresh peer timestamp
      peer_tsclock += 5;
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, f_seq(p) + lens[k], F_ACK, peer_win,
                        ts_opt(peer_tsclock, tv), no_bytes()));
      repeat (2000) @(posedge ap_clk);
    end
  endtask

  // T1: RFC 7323 PAWS - a segment with a timestamp older than TS.Recent is
  // dropped (no data delivery, no rcv_nxt advance) and answered with a
  // duplicate ACK whose TSecr still reflects the newest valid peer TSval.
  task automatic test_paws_drop();
    bytes_t p;
    bit ok, got;
    notif_t n;
    bit [15:0] sid;
    bit [31:0] toe_ts, tv, te;
    txrsp_t rsp;

    app_listen(16'd80, ok);
    do_ts_handshake(16'd80, 16'd41410, 32'h3100_0000, 32'd50_000, toe_ts);
    if (err_count) return;

    // establish TS.Recent = 50_010 with a first data segment
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, peer_ack, F_ACK, peer_win,
                      ts_opt(32'd50_010, toe_ts), gen_payload(8, 8'h21)));
    wait_notif(n, 200000, ok);
    tb_check(ok, "first data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of first 8B", p, 400000, 0);
    if (err_count) return;
    ok = get_ts_option(p, tv, te);
    toe_ts = tv;

    // in-order segment with an OLD timestamp -> PAWS reject:
    // no notification, duplicate ACK without rcv_nxt advance
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, peer_ack, F_ACK, peer_win,
                      ts_opt(32'd1_000, toe_ts), gen_payload(8, 8'h22)));
    expect_flags(F_ACK, "duplicate ACK on PAWS reject", p, 400000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq, $sformatf("PAWS dup-ACK ack=%0d (rcv_nxt not advanced)", f_ack(p)));
    ok = get_ts_option(p, tv, te);
    tb_check(ok, "PAWS dup-ACK carries timestamps");
    if (ok) tb_check(te == 32'd50_010, $sformatf("TSecr=%0d still the last valid TSval", te));
    wait_notif(n, 100000, ok);
    tb_check(!ok, "no data notification for the PAWS-rejected segment");

    // same bytes with a fresh timestamp -> accepted
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, peer_ack, F_ACK, peer_win,
                      ts_opt(32'd50_020, toe_ts), gen_payload(8, 8'h22)));
    wait_notif(n, 200000, ok);
    tb_check(ok, "fresh-timestamp segment delivered");
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of retried 8B", p, 400000, 0);
    if (!err_count) begin
      tb_check(f_ack(p) == peer_seq, "rcv_nxt advanced after valid segment");
      ok = get_ts_option(p, tv, te);
      if (ok) tb_check(te == 32'd50_020, "TS.Recent advanced to the fresh TSval");
    end
  endtask

  // R1: segments arriving in TIME_WAIT are answered with an ACK, not a RST
  // (RFC 793). Path to TIME_WAIT: app closes first (FIN), peer ACKs and sends
  // its own FIN, TOE ACKs it -> TIME_WAIT. Then a retransmitted FIN and a
  // stray pure ACK must both draw an ACK.
  task automatic test_time_wait_ack();
    bytes_t p;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    bit [31:0] fin_seq;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41500);
    if (err_count) return;

    // learn session id
    peer_send_seg(F_ACK, gen_payload(8, 8'h31));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);
    if (err_count) return;

    // active close by the TOE
    app_close(sid);
    expect_flags(F_FIN, "TOE FIN", p, 400000, 1);
    if (err_count) return;
    fin_seq = f_seq(p);
    // ACK the FIN -> TOE enters FIN_WAIT_2
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, fin_seq + 1, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (2000) @(posedge ap_clk);
    // peer FIN -> TOE ACKs, enters TIME_WAIT
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, fin_seq + 1, F_FIN | F_ACK, peer_win, no_bytes(), no_bytes()));
    peer_seq += 1;
    expect_flags(F_ACK, "ACK of peer FIN", p, 400000, 0);
    if (err_count) return;
    tb_check(f_ack(p) == peer_seq, "FIN ACK number");
    wait_notif(n, 200000, ok); // closed notification
    repeat (2000) @(posedge ap_clk);

    // (1) retransmitted FIN in TIME_WAIT -> ACK, not RST
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq - 1, fin_seq + 1, F_FIN | F_ACK, peer_win, no_bytes(), no_bytes()));
    wait_pkt(p, 400000, ok);
    tb_check(ok, "response to retransmitted FIN");
    if (ok) begin
      tb_check((f_flags(p) & F_RST) == 0, "no RST for retransmitted FIN in TIME_WAIT");
      tb_check((f_flags(p) & F_ACK) != 0, "ACK for retransmitted FIN in TIME_WAIT");
    end

    // (2) stray pure ACK in TIME_WAIT -> ACK, not RST (pre-fix: RST)
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, fin_seq + 1, F_ACK, peer_win, no_bytes(), no_bytes()));
    wait_pkt(p, 400000, ok);
    tb_check(ok, "response to stray ACK in TIME_WAIT");
    if (ok) begin
      tb_check((f_flags(p) & F_RST) == 0, "no RST for stray ACK in TIME_WAIT");
      tb_check((f_flags(p) & F_ACK) != 0, "ACK for stray ACK in TIME_WAIT");
    end
    repeat (2000) @(posedge ap_clk);
  endtask

  // R1: ISN quality - two connections must not draw the old LFSR pattern
  // (identical low 6 bits = 6'b111111) and must differ from each other
  task automatic test_isn_quality();
    bytes_t p;
    bit ok;
    bit [31:0] iss[2];
    bit [15:0] rports[2];

    app_listen(16'd80, ok);
    rports[0] = 16'd41510; rports[1] = 16'd41511;
    for (int k = 0; k < 2; k++) begin
      peer_port = rports[k]; toe_port = 16'd80;
      peer_seq  = 32'h6000_0000 + (k << 16);
      peer_win  = 16'hFFFF;
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, 0, F_SYN, 16'hFFFF, syn_options(1), no_bytes()));
      expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 100000, 1);
      if (err_count) return;
      iss[k] = f_seq(p);
      tb_info($sformatf("connection %0d ISS=%08h", k, iss[k]));
      peer_ack = iss[k] + 1;
      peer_seq += 1;
      peer_send_seg(F_ACK, no_bytes());
      repeat (1000) @(posedge ap_clk);
    end
    tb_check(iss[0] != iss[1], "ISNs differ between connections");
    tb_check(!(iss[0][5:0] == 6'h3F && iss[1][5:0] == 6'h3F),
             "low ISN bits are not the fixed pre-fix 111111 pattern");
  endtask

  // Z1: zero-window persist probe (RFC 1122 4.2.2.17). Requires the
  // timer-scaled build: TIME_50ms/1000 ~= 8 ticks -> a probe (pure ACK with
  // seq = snd_nxt - 1) every ~8k cycles while the peer advertises win=0.
  task automatic test_zero_window_probe();
    bytes_t p;
    bit ok, got;
    notif_t n;
    bit [15:0] sid;
    bit [31:0] snd_nxt;
    realtime t1, t2;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41600);
    if (err_count) return;
    peer_send_seg(F_ACK, gen_payload(8, 8'h51));
    wait_notif(n, 200000, ok);
    tb_check(ok, "data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);
    if (err_count) return;
    snd_nxt = f_seq(p);

    // peer closes its window
    peer_win = 16'h0000;
    peer_send_seg(F_ACK, no_bytes());

    // first persist probe: pure ACK at seq = snd_nxt - 1
    got = 0;
    while (!got) begin
      wait_pkt(p, 100000, ok);
      if (!ok) begin tb_error("timeout waiting for 1st persist probe"); return; end
      if (f_payload_len(p) == 0 && f_seq(p) == snd_nxt - 1) got = 1;
    end
    t1 = $realtime;
    tb_info("1st persist probe received");
    // peer still advertises zero window -> another probe ~50ms(scaled) later
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, snd_nxt, F_ACK, 16'h0000, no_bytes(), no_bytes()));
    got = 0;
    while (!got) begin
      wait_pkt(p, 100000, ok);
      if (!ok) begin tb_error("timeout waiting for 2nd persist probe"); return; end
      if (f_payload_len(p) == 0 && f_seq(p) == snd_nxt - 1) got = 1;
    end
    t2 = $realtime;
    // scaled TIME_50ms = 8 ticks ~ 8k cycles ~ 51us; allow generous margin
    tb_check((t2 - t1) < 400_000.0, $sformatf("probe spacing %.0fns ~ 50ms(scaled)", t2 - t1));
    tb_info("2nd persist probe received");

    // window opens: at most one resume probe, then silence
    peer_win = 16'h4000;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, snd_nxt, F_ACK, peer_win, no_bytes(), no_bytes()));
    wait_pkt(p, 50000, ok); // optional resume probe
    wait_pkt(p, 100000, ok);
    tb_check(!ok, "probing stops after the window opens");
    repeat (2000) @(posedge ap_clk);
  endtask

  // W1: receiver SWS avoidance (RFC 1122 4.2.3.3). The advertised window
  // right edge only advances in steps of >= 1 MSS: freeing 700B must not
  // grow the advertised window; freeing another 800B (cumulative 1500) must
  // grow it by exactly 1500 (>> win_shift).
  task automatic test_sws_avoidance();
    bytes_t p;
    bit ok, got;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    bit [15:0] w0, w1, w2;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41700);
    if (err_count) return;

    // segments sized so the app can read exactly per packet boundary
    peer_send_seg(F_ACK, gen_payload(700, 8'h71));
    wait_notif(n, 200000, ok); if (!ok) begin tb_error("notif 700"); return; end
    sid = n.sessionID; peer_seq += 700;
    peer_send_seg(F_ACK, gen_payload(800, 8'h72));
    wait_notif(n, 200000, ok); if (!ok) begin tb_error("notif 800"); return; end
    peer_seq += 800;
    peer_send_seg(F_ACK, gen_payload(1000, 8'h73));
    wait_notif(n, 200000, ok); if (!ok) begin tb_error("notif 1000"); return; end
    peer_seq += 1000;

    // ACK covering all 2500B carries the shrunken window
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for ACK of 2500B"); return; end
      if ((f_flags(p) & F_ACK) && f_ack(p) == peer_seq) begin w0 = f_win(p); got = 1; end
    end
    while (net_pkts.size() > 0) void'(net_pkts.pop_front());

    // free 700B: below one MSS -> the update ACK must advertise the SAME window
    app_read(sid, 700, rdata);
    tb_check(rdata.size() == 700, "app_read 700B");
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for update ACK (700B)"); return; end
      if ((f_flags(p) & F_ACK) && f_payload_len(p) == 0) begin w1 = f_win(p); got = 1; end
    end
    tb_check(w1 == w0, $sformatf("window unchanged after 700B freed (SWS): %0d vs %0d", w1, w0));

    // free another 800B: cumulative 1500 >= MSS -> edge advances by 1500
    app_read(sid, 800, rdata);
    tb_check(rdata.size() == 800, "app_read 800B");
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for update ACK (800B)"); return; end
      if ((f_flags(p) & F_ACK) && f_payload_len(p) == 0 && f_win(p) != w1) begin w2 = f_win(p); got = 1; end
    end
    tb_check(w2 == w1 + (16'd1500 >> 2), $sformatf("window grew by 1500>>2 (%0d -> %0d)", w1, w2));
    repeat (2000) @(posedge ap_clk);
  endtask

`ifdef NO_RX_BYPASS
  // M1: two concurrent receive gaps (RFC 2018 multi-block SACK). Sequence:
  // A[0,1k) in order, C[2k,3k) -> 1 SACK block, E[4k,5k) -> 2 blocks,
  // E2[5k,6k) extends block 1, fill B[1k,2k) -> cumulative ACK 3k + 1 block,
  // fill D[3k,4k) -> ACK 6k, no SACK; then the app reads 6000 intact bytes.
  task automatic test_multi_gap_sack();
    bytes_t p;
    bit ok, got;
    notif_t n;
    byte unsigned rdata[$];
    bit [15:0] sid;
    bit [31:0] base, s0, e0, s1, e1;
    int nblk;

    app_listen(16'd80, ok);
    peer_port = 16'd41800; toe_port = 16'd80;
    peer_seq  = 32'h7000_0000; peer_win = 16'hFFFF;
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, 0, F_SYN, 16'hFFFF, syn_options_sack(), no_bytes()));
    expect_flags(F_SYN | F_ACK, "SYN-ACK", p, 100000, 1);
    if (err_count) return;
    peer_ack = f_seq(p) + 1;
    peer_seq += 1;
    peer_send_seg(F_ACK, no_bytes());
    base = peer_seq;

    // A: in order
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      base, peer_ack, F_ACK, peer_win, no_bytes(), gen_payload(1000, 8'h81)));
    wait_notif(n, 200000, ok);
    tb_check(ok && n.length == 1000, "A notified");
    if (!ok) return;
    sid = n.sessionID;

    // C: opens gap 0
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      base + 2000, peer_ack, F_ACK, peer_win, no_bytes(), gen_payload(1000, 8'h83)));
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for dup ACK (C)"); return; end
      nblk = get_sack_blocks(p, s0, e0, s1, e1);
      if (f_payload_len(p) == 0 && nblk > 0) got = 1;
    end
    tb_check(nblk == 1, "one SACK block after C");
    tb_check(s0 == base + 2000 && e0 == base + 3000, "block 0 = [2000,3000)");
    tb_check(f_ack(p) == base + 1000, "dup ACK number (C)");

    // E: opens gap 1 -> two blocks
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      base + 4000, peer_ack, F_ACK, peer_win, no_bytes(), gen_payload(1000, 8'h85)));
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for dup ACK (E)"); return; end
      nblk = get_sack_blocks(p, s0, e0, s1, e1);
      if (f_payload_len(p) == 0 && nblk == 2) got = 1;
    end
    tb_check(s0 == base + 2000 && e0 == base + 3000, "block 0 still [2000,3000)");
    tb_check(s1 == base + 4000 && e1 == base + 5000, "block 1 = [4000,5000)");

    // E2: extends range 1
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      base + 5000, peer_ack, F_ACK, peer_win, no_bytes(), gen_payload(1000, 8'h86)));
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for dup ACK (E2)"); return; end
      nblk = get_sack_blocks(p, s0, e0, s1, e1);
      if (f_payload_len(p) == 0 && nblk == 2 && e1 == base + 6000) got = 1;
    end
    tb_check(s1 == base + 4000, "block 1 extended to [4000,6000)");

    // fill B: cumulative ACK jumps over range 0, range 1 remains
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      base + 1000, peer_ack, F_ACK, peer_win, no_bytes(), gen_payload(1000, 8'h82)));
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for ACK (B fill)"); return; end
      if (f_payload_len(p) == 0 && f_ack(p) == base + 3000) got = 1;
    end
    nblk = get_sack_blocks(p, s0, e0, s1, e1);
    tb_check(nblk == 1, "one SACK block after filling gap 0");
    tb_check(s0 == base + 4000 && e0 == base + 6000, "remaining block = [4000,6000)");

    // fill D: everything contiguous
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      base + 3000, peer_ack, F_ACK, peer_win, no_bytes(), gen_payload(1000, 8'h84)));
    got = 0;
    while (!got) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for ACK (D fill)"); return; end
      if (f_payload_len(p) == 0 && f_ack(p) == base + 6000) got = 1;
    end
    nblk = get_sack_blocks(p, s0, e0, s1, e1);
    tb_check(nblk == 0, "no SACK block once contiguous");
    peer_seq = base + 6000;

    // application data integrity across the reassembled 6000 bytes
    app_read(sid, 6000, rdata);
    tb_check(rdata.size() == 6000, "app_read 6000B");
    begin
      bit match = 1;
      for (int i = 0; i < 6000; i++) begin
        byte unsigned exp;
        exp = ((8'h81 + i/1000) + (i % 1000)) & 8'hFF;
        if (rdata[i] !== exp) match = 0;
      end
      tb_check(match, "reassembled data intact across both gaps");
    end
    repeat (2000) @(posedge ap_clk);
  endtask
`endif

  // C1: congestion avoidance byte counting (RFC 3465). After fast recovery
  // deflates cwnd to ssthresh=4900, each full-MSS ACK adds 1460 acked bytes
  // to the accumulator; cwnd grows by one MSS only when a whole cwnd worth
  // of bytes has been acked: 4900,4900,4900,6360,6360,6360,6360,7820.
  task automatic test_ca_growth();
    bytes_t p, pay;
    bit ok, ok2;
    notif_t n;
    bit [15:0] sid;
    txrsp_t rsp;
    bit [31:0] d0_seq, d_seq;
    int got;
    int unsigned cw;
    int unsigned exp_cw[8];

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd41900);
    if (err_count) return;
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);

    // enter and leave fast recovery (as in test_fast_recovery) to get a low
    // ssthresh so congestion avoidance is actually exercised
    for (int i = 0; i < 8; i++) begin
      pay = gen_payload(1400, 8'h30 + i[7:0]);
      app_send(sid, pay, rsp);
    end
    got = 0;
    while (got < 8) begin
      wait_pkt(p, 400000, ok2);
      if (!ok2) begin tb_error("timeout collecting data segments"); return; end
      if (f_payload_len(p) == 1400) begin
        if (got == 0) d0_seq = f_seq(p);
        got++;
      end
    end
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_seq + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (3000) @(posedge ap_clk);
    for (int i = 0; i < 4; i++) begin
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, d0_seq + 1400, F_ACK, peer_win, no_bytes(), no_bytes()));
      repeat (100) @(posedge ap_clk);
    end
    repeat (3000) @(posedge ap_clk);
    net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                      peer_seq, d0_seq + 11200, F_ACK, peer_win, no_bytes(), no_bytes()));
    repeat (3000) @(posedge ap_clk);
    cw = dut.tx_sar_table_U0.tx_table_cong_window_V_U.ram[sid];
    tb_check(cw == 4900, $sformatf("cwnd=%0d expected 4900 after recovery exit", cw));
    while (net_pkts.size() > 0) void'(net_pkts.pop_front());

    // congestion avoidance: 8 full-MSS round trips
    exp_cw[0] = 4900; exp_cw[1] = 4900; exp_cw[2] = 4900; exp_cw[3] = 6360;
    exp_cw[4] = 6360; exp_cw[5] = 6360; exp_cw[6] = 6360; exp_cw[7] = 7820;
    for (int k = 0; k < 8; k++) begin
      pay = gen_payload(1460, 8'h90 + k[7:0]);
      app_send(sid, pay, rsp);
      tb_check(rsp.error == 0, $sformatf("CA app_send %0d accepted", k));
      got = 0;
      while (!got) begin
        wait_pkt(p, 400000, ok2);
        if (!ok2) begin tb_error($sformatf("timeout waiting for CA segment %0d", k)); return; end
        if (f_payload_len(p) == 1460) begin d_seq = f_seq(p); got = 1; end
      end
      net_send(make_tcp(PEER_IP, TOE_IP, peer_port, toe_port,
                        peer_seq, d_seq + 1460, F_ACK, peer_win, no_bytes(), no_bytes()));
      repeat (3000) @(posedge ap_clk);
      cw = dut.tx_sar_table_U0.tx_table_cong_window_V_U.ram[sid];
      tb_check(cw == exp_cw[k], $sformatf("CA step %0d: cwnd=%0d expected %0d", k, cw, exp_cw[k]));
    end
    repeat (2000) @(posedge ap_clk);
  endtask

  // C3: retransmission gives up after 7 data retries (SYN keeps 4).
  // Requires the timer-scaled build; the exponential ladder sums to ~10M
  // cycles, so this test (like keepalive) runs with the 40M-cycle watchdog.
  task automatic test_rt_abort();
    bytes_t p, pay;
    bit ok;
    notif_t n;
    bit [15:0] sid;
    bit [31:0] d_seq;
    int rtx, nn;
    txrsp_t rsp_dummy;

    app_listen(16'd80, ok);
    do_passive_handshake(16'd80, 16'd42000);
    if (err_count) return;
    peer_send_seg(F_ACK, gen_payload(8, 8'h01));
    wait_notif(n, 200000, ok);
    tb_check(ok, "initial data notification");
    if (!ok) return;
    sid = n.sessionID;
    peer_seq += 8;
    expect_flags(F_ACK, "ACK of 8B", p, 400000, 0);
    if (err_count) return;

    // 200B that the peer never ACKs
    pay = gen_payload(200, 8'h44);
    app_send(sid, pay, rsp_dummy);
    ok = 0;
    while (!ok) begin
      wait_pkt(p, 400000, ok);
      if (!ok) begin tb_error("timeout waiting for original segment"); return; end
      if (f_payload_len(p) != 200) ok = 0;
    end
    d_seq = f_seq(p);

    // exactly 7 retransmissions of the same segment
    for (rtx = 1; rtx <= 7; rtx++) begin
      ok = 0;
      while (!ok) begin
        wait_pkt(p, 6_000_000, ok);
        if (!ok) begin tb_error($sformatf("timeout waiting for retransmission %0d", rtx)); return; end
        if (f_payload_len(p) != 200) ok = 0; // skip keepalive/pure ACKs
      end
      tb_check(f_seq(p) == d_seq, $sformatf("rtx %0d seq", rtx));
      tb_info($sformatf("retransmission %0d seen", rtx));
    end

    // then the connection is aborted: closed notification + CAM delete,
    // and no further data retransmissions
    ok = 0; nn = 0;
    while (!ok && nn < 3) begin
      wait_notif(n, 6_000_000, ok);
      if (ok && !n.closed) ok = 0;
      nn++;
    end
    tb_check(ok && n.closed, "closed notification after retry exhaustion");
    if (ok) tb_check(n.sessionID == sid, "notification session id");
    nn = 0;
    while (cam_delete_count == 0 && nn < 200_000) begin @(posedge ap_clk); nn++; end
    tb_check(cam_delete_count > 0, "CAM DELETE issued on abort");
    ok = 1;
    while (ok) begin
      wait_pkt(p, 1_000_000, ok);
      if (ok && f_payload_len(p) == 200) begin
        tb_check(0, "no further retransmissions after abort");
        return;
      end
    end
  endtask

  //---------------------------------------------------------------------------
  // Main
  //---------------------------------------------------------------------------
  initial begin
    s_tcp_tvalid <= 0; s_tcp_tdata <= '0; s_tcp_tkeep <= '0; s_tcp_tstrb <= '0; s_tcp_tlast <= 0;
    listen_req_tvalid <= 0; rxreq_tvalid <= 0; open_req_tvalid <= 0; close_req_tvalid <= 0;
    txmeta_tvalid <= 0; txdata_tvalid <= 0; txdata_tlast <= 0;
    txdata_tdata <= '0; txdata_tkeep <= '0; txdata_tstrb <= '0;

    if (!$value$plusargs("TESTNAME=%s", testname))
      testname = "passive_open";

    if ($test$plusargs("DUMP")) begin
      $dumpfile("tb_toe_top.vcd");
      $dumpvars(1, tb_toe_top);
    end

    // reset
    ap_rst_n = 0;
    repeat (100) @(posedge ap_clk);
    ap_rst_n = 1;
    repeat (100) @(posedge ap_clk);

    $display("=====================================================");
    $display("  TOE RTL TB - running test: %s", testname);
    $display("=====================================================");

    case (testname)
      "passive_open"   : test_passive_open();
      "active_open"    : test_active_open();
      "rx_out_of_order": test_rx_out_of_order();
      "fast_retransmit": test_fast_retransmit();
      "rst_handling"   : test_rst_handling();
      "zero_window"    : test_zero_window();
      "port_release"   : test_port_release();
      "opt_parser_robust": test_opt_parser_robust();
      "buffer_wrap"    : test_buffer_wrap();
      "window_update"  : test_window_update();
      "pure_fin_close" : test_pure_fin_close();
      "mss_option"     : test_mss_option();
      "tx_segmentation": test_tx_segmentation();
      "fast_recovery"  : test_fast_recovery();
      "sack_selective_rtx": test_sack_selective_rtx();
      "half_close"     : test_half_close();
`ifdef NO_RX_BYPASS
      "rx_ooo_overlap" : test_rx_ooo_overlap();
      "multi_gap_sack" : test_multi_gap_sack();
      "seq_window_check": test_seq_window_check();
      "sack_rx_gen"    : test_sack_rx_gen();
`endif
      "rto_retransmit" : test_rto_retransmit();
      "fin_retransmit" : test_fin_retransmit();
      "adaptive_rto"   : test_adaptive_rto();
      "keepalive"      : test_keepalive();
      "ts_negotiate"   : test_ts_negotiate();
      "paws_drop"      : test_paws_drop();
      "time_wait_ack"  : test_time_wait_ack();
      "isn_quality"    : test_isn_quality();
      "zero_window_probe" : test_zero_window_probe();
      "sws_avoidance"  : test_sws_avoidance();
      "ca_growth"      : test_ca_growth();
      "rt_abort"       : test_rt_abort();
      default: tb_error($sformatf("unknown TESTNAME '%s'", testname));
    endcase

    repeat (100) @(posedge ap_clk);
    $display("=====================================================");
    if (err_count == 0) $display("  TEST %s : PASS", testname);
    else                $display("  TEST %s : FAIL (%0d errors)", testname, err_count);
    $display("=====================================================");
    $finish;
  end

  // global watchdog (keepalive needs ~18M cycles of protocol silence)
  initial begin
    #1;
    if (testname == "keepalive" || testname == "rt_abort") #(CLK_PERIOD * 40_000_000);
    else                                                   #(CLK_PERIOD * 10_000_000);
    $display("[%0t] TB-ERROR: global watchdog expired", $time);
    $display("  TEST %s : FAIL (watchdog)", testname);
    $finish;
  end

endmodule
