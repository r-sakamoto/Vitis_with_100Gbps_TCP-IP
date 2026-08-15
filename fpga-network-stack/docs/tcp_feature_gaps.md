# TOE (TCP Offload Engine) vs Linux TCP 機能ギャップ分析

対象: `fpga-network-stack/hls/toe/` (ETH Zurich TOE, 現行ビルド構成:
`MSS=1460, MAX_SESSIONS=1000, DATA_WIDTH=64bit, TCP_NODELAY=1,
RX_DDR_BYPASS=1, FAST_RETRANSMIT=1, WINDOW_SCALE=1(shift=2固定)`)

ソースコードの精査(rx_engine / tx_engine / tx_sar_table / rx_sar_table /
retransmit_timer / probe_timer / close_timer / session_lookup_controller /
port_table / state_table / ack_delay / tx_app_* / rx_app_*)、および
[fpgasystems/fpga-network-stack の GitHub issue](https://github.com/fpgasystems/fpga-network-stack/issues)
(#19, #21, #26, #31, #39, #42, #43, #44) に基づく。

凡例: 優先度 ★★★=相互運用・スループットに直接影響 / ★★=堅牢性・性能改善 / ★=セキュリティ・運用性

---

## 1. TCPオプション(RFC 793 / 7323 / 2018)

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| MSSオプション(受信) | **実装済** (F4+P1, `mss_option`/`tx_segmentation`テスト): 実効MSS=min(peer,local)をセッション毎に保存し、初送・再送ともMSS超過書込みをDDR読出しで分割送信(バイパス複製ストリームは破棄モードで整合)。PMTUDは未 | 相手MSSとPMTUからMSSを決定 | ★(残: PMTUD) |
| Window Scale(受信) | SYN時のみパース。**自側shiftは0か2の固定** (`rx_engine.cpp:1366,1429`)。相手shiftは**14超の値を無検証で格納** → `tx_sar_table.cpp:121` でビット範囲オーバーフロー(issue #42) | shift 0〜14を交渉、バッファサイズから導出 | ★★★ |
| SACK / SACK-Permitted | **なし**(生成・解釈とも) | あり(選択再送・D-SACK) | ★★★ |
| Timestamps (RFC 7323) | **なし** → PAWSなし、RTT計測なし | あり | ★★★ |
| ECN (ECE/CWR/ECT) | **なし**。フラグは常時0固定 (`toe_internals.hpp:522`) | あり | ★ |
| オプションパーサの正当性 | dataOffsetの単位バグ(ワード vs バイト混在, issue #43)、不正オプション列でRXパイプラインがストールし得る (`rx_engine.cpp:645-700,730-738`) | — | ★★★(バグ修正) |
| PSH / URG | PSHは常に0、URG・緊急ポインタ未対応 | あり | ★ |

## 2. 再送・RTT・輻輳制御(RFC 6298 / 5681)

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| RTT計測 / 適応RTO | **実装済** (P2, `adaptive_rto`テスト): 新モジュール`rtt_estimator`がKarn方式でRTT計測、RFC 6298のSRTT/RTTVAR→RTO=clamp(SRTT+4V, 250ms, 30s)。retransmit_timerはRTO×2^retriesの指数バックオフ化。Timestampsオプション(PAWS含む)は未 | SRTT/RTTVARによるRFC 6298、最小RTO 200ms | ★(残: TSopt) |
| 再送回数上限 | 4回再送で**RSTも送らずセッション破棄** (`retransmit_timer.cpp:166-183`) | tcp_retries2(15回)、RST送信 | ★★ |
| 再送方式 | go-back-N(ackd以降全部)、1回のRTOで最大4セグメント (`tx_engine.cpp:382-396`) | SACKベースの選択再送 | ★★★ |
| Fast Retransmit | dup ACK==3 ちょうどで1回のみ。**cwnd/ssthresh縮小なし**(fast recoveryなし) (`rx_engine.cpp:1277-1281`) | RFC 5681 fast recovery、partial ACK処理 | ★★★ |
| 輻輳制御 | slow start + 「+365バイト/ACK」の线形増加(ハードコード)。損失時はcwnd=14600固定にリセット (`rx_engine.cpp:1057-1064`, `tx_sar_table.cpp:95`) | CUBIC(デフォルト)ほか、ABC、pacing | ★★ |
| dup ACK判定 | ウィンドウ不変チェックなし、**古いACKでもcwnd増加**・カウンタリセット (`rx_engine.cpp:1052-1067`) | RFC 5681準拠 | ★★ |
| シーケンス番号の巡回演算 | ACK窓ガード1箇所以外は**素の32bit比較**(wrap非対応) (`rx_engine.cpp:1129他`) | 全比較がmod 2^32 | ★★ |

## 3. 受信処理・フロー制御

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| 順序入替(OOO)受信 | RX_DDR_BYPASS=1 (現行ビルド)では**完全ドロップ**(`rx_engine.cpp:1249-1274`)。非bypass時も単一ギャップのみ、重複セグメントでハング可能性 (`rx_engine.cpp:1197-1230`) | 完全な再構成キュー | ★★★ |
| RFC 793受理判定 | **一部実装済** (F2, `seq_window_check`テスト): 非bypassのOOO受理を広告ウィンドウ内に制限(バッファ別名書込み防止)、OOO系比較をmod 2^32化(seq_lt等ヘルパ追加)。ACK側の全面的なwrap対応は未 | あり | ★(残) |
| ウィンドウ更新通知 | **実装済** (F1, `window_update`テスト): app読出しでappdが進むとACKイベントを発行しwindow update ACKを送出 | 即時window update | 完了 |
| ゼロウィンドウプローブ | TCP_NODELAY=1ビルドでは**probeタイマが恒久無効**(`tx_engine.cpp:166-171`)。有効時も1バイトプローブでなく全再送 | RFC 1122準拠persist | ★★ |
| Delayed ACK | 64µs固定(Linuxの40msの約1/600)。イベント負荷で無限延期あり得る (`ack_delay.cpp`) | 40ms/クオックACK | ★ |
| SWS回避 | 送受とも**なし**(受信側クランプはコメントアウト `tx_engine.cpp:936-940`) | あり | ★★ |
| Keepalive | **なし**(タイマ自体が存在しない) | SO_KEEPALIVE | ★★ |

## 4. コネクション管理・ステートマシン

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| ハーフクローズ | **不可**。相手FIN受信で即座に自FINを送りLAST_ACKへ(CLOSE_WAITスキップ, `rx_engine.cpp:1508-1512`)。FIN後にappは送信不能 | shutdown(SHUT_WR)対応 | ★★ |
| FIN_WAIT_2 | 未使用(FIN_WAIT_1のACKで遷移しない, `rx_engine.cpp:1321-1323`)。TIME_WAIT中のACKに**RSTを返す** (`rx_engine.cpp:1334-1344`) | 準拠 | ★★ |
| ACKなしFIN | **実装済** (F3, `pure_fin_close`テスト): 純FINもFIN+ACKと同様に処理(ACK番号依存処理はACKフラグでガード) | 処理する | 完了 |
| 同時オープン | 非対応(SYN_SENT中のSYNをパッシブオープン扱い) | 対応 | ★ |
| TIME_WAIT | 60秒固定(2MSLでない)。TIME_WAIT中の同タプルSYN再利用不可 | 60s + 再利用(RFC 1122) | ★ |
| listen解除 | **不可**(stop-listeningパスがコメントアウト, `rx_app_if.cpp:79-82`)。port>=32768はlistne不可、二重bind検出なし | close/bind準拠 | ★ |
| ISN生成 | **固定シード(0x562301af)の決定的LFSR**、下位6bit常に0x3F (`tx_engine.cpp:82,488-490`) | RFC 6528(秘密鍵+4タプルハッシュ+時刻) | ★★ |
| SYNバックログ制限 | なし。1000個の偽SYNで**全セッション枯渇**(SYN flood無防備) | syncookies等 | ★ |
| RST検証 | seq==recvd完全一致のみ(RFC 5961のchallenge ACKなし) | RFC 5961 | ★ |
| マルチホーム | 3タプル(theirIp,myPort,theirPort)キーで**ローカルIPは1つのみ** | 4タプル | ★ |

## 5. 実装バグ(機能追加とは別に修正が必要)

| # | 状態 | 内容 | 場所 |
|---|---|---|---|
| B1 | **修正済** (`port_release`テストで検証) | エフェメラルポートが**解放されない**(エンディアン不一致: `releaseTuple.myPort`がスワップ済のままport_tableの`>=32768`判定に使われる) → active openがいずれ枯渇。release時に`reverse()`を追加 | `session_lookup_controller.cpp` reverseLookupTableInterface |
| B2 | **修正済** (NODELAY=0パス、比較の向きを訂正) | tx_sar_tableの`usableWindow`計算の比較が逆(`minWindow < usedLength`)。NODELAY=1では潜在 | `tx_sar_table.cpp:132-136` |
| B3 | **修正済** (NODELAY=0構成のcsynth完走を確認) | TCP_NODELAY=0(Nagle)ビルドは**コンパイル不能**(`txSar`未宣言、セミコロン欠落) | `tx_engine.cpp:208-331,1427` |
| B4 | **修正済** (`buffer_wrap`テストで検証) | (a)再送時のバッファwrap: 2つ目の読み出しコマンドの先頭アドレスが0固定(issue #31)、(b)`rxAppMemAccessBreakdown`が64KB境界ハードコード分割で誤アドレス継続 | `tx_engine.cpp` txEngMemAccessBreakdown / `toe.cpp` rxAppMemAccessBreakdown |
| B5 | **修正済** (`opt_parser_robust`テストで検証) | オプションパーサのdataOffset単位バグ(issue #43)、不正オプション(len=0)での無限ループ=RXパス恒久停止、win_shift>14未クランプ(issue #42)。バイト単位カウント+堅牢化+14クランプに書換え | `rx_engine.cpp` parse_optional_header_fields |
| B6 | **修正済** (`rx_ooo_overlap`テストで検証) | OOOギャップ充填時の重複セグメントで分岐が無くパイプラインストール(非bypass構成) | `rx_engine.cpp` ギャップ充填分岐 |
| B7 | **修正済** (`fin_retransmit`テストで検証。RT再送長からFINファントムバイトを除外) | FIN再送タイマのイベント型がRT(FIXMEコメントあり) | `tx_engine.cpp:643-644` |
| B8 | **修正済** (`#ifndef __SYNTHESIS__`ガードを28ブロックに適用) | 合成対象コードに`std::cout`が無ガードで残存 | `tx_engine.cpp`他多数 |

## 6. 実装優先順位の提案(Linux同等化ロードマップ)

1. **フェーズ1(正しさ・相互運用)**: B1〜B6のバグ修正、MSSオプション受信対応、
   ウィンドウ更新ACK送出、汎用シーケンス窓チェック+wrap対応比較、ACKなしFIN処理
2. **フェーズ2(性能)**: RTT計測(Timestampsオプション)+RFC 6298適応RTO、
   RFC 5681 fast recovery(ssthresh/cwnd正規化、MSS依存定数除去)、
   受信OOOバッファリング(bypass構成でも単一ギャップ保持)、SWS回避
3. **フェーズ3(SACK)**: SACK-Permitted/SACKブロック生成・解釈、SACKベース選択再送
4. **フェーズ4(堅牢性)**: keepalive、persist正規化、half-close(CLOSE_WAIT)、
   FIN_WAIT_2、RFC 6528 ISN、RFC 5961、TIME_WAIT再利用、listen解除

## 7. 検証環境

- RTLシミュレーション: `sim/toe_rtl/`(xsim + SystemVerilog TB、`sim/toe_rtl/README.md`参照)
- タイマ系(RTO/persist/TIME_WAIT)の検証はタイマ定数を縮小したシミュレーション用
  ビルドが必要(`toe_internals.hpp` の `TIME_*` スケール化。TODO)
