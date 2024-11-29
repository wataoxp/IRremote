/*
 * IRremote.h
 *
 *  Created on: Nov 16, 2024
 *      Author: wataoxp
 */

#ifndef INC_IRREMOTE_H_
#define INC_IRREMOTE_H_

#include "main.h"

#define IR_DECODE_READY 0
#define IR_ACK 0x06
#define IR_NACK 0x15
#define IR_REPEAT 0x20

/* Falling Edge Exti Timing */
#define IR_MIN_TIMING 10000 //10000us.RepeatCode Limit
#define IR_MAX_TIMING 14000 //14000us.LeaderCode Limit
#define IR_LEADER_TIMING 13000 //13000us.LeaderCode
#define IR_REPEAT_TIMING 11000 //11000us.RepeatCode
#define NEC_HIGH 2000
#define NEC_LOW 1400

/* Polling Status */
#define IR_P_REPEAT 1
#define IR_P_REPEAT_TIMING 4000 //4000us

typedef enum{
	StartLeader,
	EndLeader,
	ErrorLeader,
	ReadTime,
}IRstatus;

typedef enum{
	Init,
	Save,
}INTmode;

typedef struct{
	uint8_t bit31;
	uint8_t bit24;
	uint8_t bit16;
	uint8_t bit8;
}ConvertLSB;

void Sleep200ms(TIM_TypeDef *TIMsleep);
/*
 * TIM14をセット。200ms待機
 * 他の処理を実行している状態を想定しています。
 */
uint32_t RecieveIR(TIM_TypeDef *TIMx);
/*
 * TIM16をセット。ポーリングで受信と解析を行います。
 * 基本的に関数の先頭で受信モジュールの立ち下がりエッジを待ち続けているので、他の処理と並行することは困難です。
 * ただしタイマーしか使用しないので受信と解析のテストには適した関数です。
 */
void RecieveIR_IT(TIM_TypeDef *TIMx,uint32_t *Binary,uint8_t *Flag,uint8_t mode);
/*
 * TIM16をセット。割り込みで受信と解析を行います。
 * 引数Binaryが受信データを格納、Flagはmain関数に進行状況を伝える為の変数です。
 * 最初にmain関数からこの関数を呼び、BinaryとFlagのアドレスをセットしてください。
 * 内部のstaticポインタがアドレスを記憶し、ポインタを介して値を代入します。
 *
 * 意図しない形での多重割り込みを防ぐ事を期待して関数の最初と最後で割り込みの禁止・許可を行っています。
 */
void BinaryToHex(ConvertLSB *LSB,uint32_t *Binary);
/*
 * 受信関数がMSB並びで格納したビット列をLSB列で、かつ8ビット変数4つに分解します。
 */
void DataReset(ConvertLSB *LSB);
/*
 * LSB構造体の初期化・リセット関数
 * memset()を使っているのでこの関数は使いませんが、一応残しておきます。
 */

/* *** NECフォーマットについて ***
 * 本プログラムはNECフォーマットの赤外線リモコンを想定したものになっています。
 * また赤外線受信においてはOSRB38C99Aを利用しています。
 * 同モジュールにおいては送信側の赤外線が「点灯」したときに「Low」が出力されます。
 * ですので特に割り込みを使う場合、立ち下がりエッジを検出することで1周期の時間をチェックするようにします。
 *
 * NECフォーマットは以下の形で信号が送信されます。
 *
 * LeaderCode	通信の開始を伝える
 * 9000us(9ms)間のLow、4000us(4ms)のHigh
 * 割り込みを使う場合、初回の立ち下がりから次の立ち下がりまでの時間が13000us程度であれば正しいデータと判断できます。
 *
 * AｄｄｒｅｓｓCｏｄｅ 送信側個体のアドレスを伝える。16ビット
 *
 * DataCode リモコンであればどのボタンが押されたかの情報を伝える。16ビット
 *
 * Address、Dataともに0と1は以下の周期で判定できます。
 * Low時間は共通で約600us
 * High時間は 0のとき500us。1のとき1600us。
 * なので1ビットずつ周期を測り、2000us以上であれば1であると判断するようにしています。
 *
 * RepeatCode
 * 長押し時に送信されるデータです。
 * LeaderCodeと同じ9000usのLowから始まり、2000usのHighが続きます。
 * ロジックアナライザでの測定をした限り、データ受信後から50ms程度の時間に意図しないRepeatCodeが送られていました。
 * ですのでデータ受信後は一定時間次のデータを無視するなど、チャタリング(debounce)対策のようなことが必要になるかもしれません。
 *
 * *** 周辺環境の影響 ***
 * 受信モジュールによるのかもしれませんが、テスト中は可能な限り受信モジュール周辺を暗い環境にすることを推奨します。
 * 特に直射日光が当たる環境でのテストは非常に困難です。
 * 私はLED照明のほぼ直下でテストを行っていましたが、それでもまれに無効な信号(ノイズ)が検知されました。
 * 参考になれば幸いです。
 * */

#endif /* INC_IRREMOTE_H_ */
