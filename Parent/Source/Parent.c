/****************************************************************************
 * (C) Tokyo Cosmos Electric, Inc. (TOCOS) - 2013 all rights reserved.
 *
 * Condition to use:
 *   - The full or part of source code is limited to use for TWE (TOCOS
 *     Wireless Engine) as compiled and flash programmed.
 *   - The full or part of source code is prohibited to distribute without
 *     permission from TOCOS.
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "utils.h"

#include "Parent.h"
#include "config.h"
#include "Version.h"

#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"

#include "serialInputMgr.h"
#include "btnMgr.h"
#include "AddrKeyAry.h"

#include "common.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
#define ToCoNet_USE_MOD_NWK_LAYERTREE // Network definition
#define ToCoNet_USE_MOD_NBSCAN // Neighbour scan module
#define ToCoNet_USE_MOD_NBSCAN_SLAVE // Neighbour scan slave module
#define ToCoNet_USE_MOD_CHANNEL_MGR
#define ToCoNet_USE_MOD_NWK_MESSAGE_POOL
#define ToCoNet_USE_MOD_DUPCHK

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define V_PRINTF(...) vfPrintf(&sSerStream,__VA_ARGS__)
#define V_PUTCHAR(c) (&sSerStream)->bPutChar((&sSerStream)->u8Device, c)

#define TOCONET_DEBUG_LEVEL 0

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/**
 * アプリケーション基本データ構造体
 */
typedef struct {
	// MAC
	uint8 u8channel;

	// LED Counter
	uint32 u32LedCt;

	// Network context
	tsToCoNet_Nwk_Context *pContextNwk;
	tsToCoNet_NwkLyTr_Config sNwkLayerTreeConfig;

	uint16 u16LedDur_ct; //! LED点灯カウンタ

	uint16 u16frame_count; //! シーケンスカウンタ
} tsAppData;

// 子機の発報情報を保存するデータベース
tsAdrKeyA_Context sEndDevList;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vInitHardware(int f_warm_start);
static void vSerialInit(void);
static void vHandleSerialInput(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsAppData sAppData; // application information
tsFILE sSerStream; // serial output context
tsSerialPortSetup sSerPort; // serial port queue
tsSerCmd sSerCmd; // serial command parser

/****************************************************************************/
/***        ToCoNet Functions                                             ***/
/****************************************************************************/
/**
 * アプリケーションの起動時の処理
 * - ネットワークの設定
 * - ハードウェアの初期化
 */
PUBLIC void cbAppColdStart(bool_t bAfterAhiInit) {
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.

		// Register modules
		ToCoNet_REG_MOD_ALL();

	} else {
		// disable brown out detect
		//vAHI_BrownOutConfigure(0, //0:2.0V 1:2.3V
		//		FALSE, FALSE, FALSE, FALSE);

		// clear application context
		memset(&sAppData, 0x00, sizeof(sAppData));
		memset(&sSerCmd, 0x00, sizeof(sSerCmd));
		ADDRKEYA_vInit(&sEndDevList);
		SPRINTF_vInit128();

		sAppData.u8channel = CHANNEL;

		// ToCoNet configuration
		sToCoNet_AppContext.u32AppId = APP_ID;
		sToCoNet_AppContext.u8Channel = CHANNEL;
		sToCoNet_AppContext.u32ChMask = CHMASK;

		sToCoNet_AppContext.bRxOnIdle = TRUE;
		sToCoNet_AppContext.u8TxMacRetry = 1;

		// Register
		ToCoNet_Event_Register_State_Machine(vProcessEvCore);

		// Others
		vInitHardware(FALSE);
	}
}

/**
 * スリープ復帰時の処理（本アプリケーションでは処理しない)
 * @param bAfterAhiInit
 */
void cbAppWarmStart(bool_t bAfterAhiInit) {
	cbAppColdStart(bAfterAhiInit);
}

/**
 * メイン処理
 * - シリアルポートの処理
 */
void cbToCoNet_vMain(void) {
	/* handle uart input */
	//vHandleSerialInput();
}

/**
 * ネットワークイベント。
 * - E_EVENT_TOCONET_NWK_START\n
 *   ネットワーク開始時のイベントを vProcessEvCore に伝達
 *
 * @param eEvent
 * @param u32arg
 */
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
	switch (eEvent) {
	case E_EVENT_TOCONET_NWK_START:
		// send this event to the local event machine.
		ToCoNet_Event_Process(eEvent, u32arg, vProcessEvCore);
		break;
	default:
		break;
	}
}

/**
 * 子機または中継機を経由したデータを受信する。
 *
 * - アドレスを取り出して、内部のデータベースへ登録（メッセージプール送信用）
 * - UART に指定書式で出力する
 *   - 出力書式\n
 *     ::(受信元ルータまたは親機のアドレス):(シーケンス番号):(送信元アドレス):(LQI)<CR><LF>
 *
 * @param pRx 受信データ構造体
 */
PUBLIC void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {
	// ここでUART出力
	// CRC16確認する
	pRx->auData[pRx->u8Len] = 0;
}

/**
 * 送信完了時のイベント
 * @param u8CbId
 * @param bStatus
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	return;
}

/**
 * ハードウェア割り込みの遅延実行部
 *
 * - BTM による IO の入力状態をチェック\n
 *   ※ 本サンプルでは特別な使用はしていない
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) { }

/**
 * ハードウェア割り込み
 * - 処理なし
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 * @return
 */
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	return FALSE;
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/**
 * ハードウェアの初期化
 * @param f_warm_start
 */
static void vInitHardware(int f_warm_start) {
	// シリアルポートの初期化
	vSerialInit();
	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(TOCONET_DEBUG_LEVEL);

	// IO の設定
	vPortAsOutput(PORT_KIT_LED1);
	vPortAsOutput(PORT_KIT_LED2);
	vPortAsOutput(PORT_KIT_LED3);
	vPortAsOutput(PORT_KIT_LED4);
	vPortSetHi(PORT_KIT_LED1);
	vPortSetHi(PORT_KIT_LED2);
	vPortSetHi(PORT_KIT_LED3);
	vPortSetHi(PORT_KIT_LED4);
}

/**
 * UART の初期化
 */
static void vSerialInit(void) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[1532];
	static uint8 au8SerialRxBuffer[512];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = UART_BAUD;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInit(&sSerPort);

	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT;
}

/**
 * シリアルポートの入力
 * - + + + シーケンスにより１バイトコマンドを入力できるようにする
 */
static void vHandleSerialInput(void) { }

/**
 * アプリケーション主要処理
 * - E_STATE_IDLE\n
 *   ネットワークの初期化、開始
 *
 * - E_STATE_RUNNING\n
 *   - データベースのタイムアウト処理
 *   - 定期的なメッセージプール送信
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {

	if (eEvent == E_EVENT_TICK_SECOND) {
	}

	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			//V_PRINTF(LB"[E_STATE_IDLE]");

#ifdef USE_AES
			ToCoNet_bRegisterAesKey((void*)au8EncKey, NULL);
#endif

			// Configure the Network
			sAppData.sNwkLayerTreeConfig.u8Layer = 0;
			sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_PARENT;
			sAppData.pContextNwk =
					ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
			if (sAppData.pContextNwk) {
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);
			}

		} else if (eEvent == E_EVENT_TOCONET_NWK_START) {
			ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		} else {
			;
		}
		break;

	case E_STATE_RUNNING:
		if (eEvent == E_EVENT_NEW_STATE) {
			//V_PRINTF(LB"[E_STATE_RUNNING]");
		} else if (eEvent == E_EVENT_TICK_SECOND) {
			static uint8 u8Ct_s = 0;
			int i;

			// 定期クリーン（タイムアウトしたノードを削除する）
			ADDRKEYA_bFind(&sEndDevList, 0, 0);

			// 共有情報（メッセージプール）の送信
			//   - OCTET: 経過秒 (0xFF は終端)
			//   - BE_DWORD: 発報アドレス
			if (++u8Ct_s > PARENT_MSGPOOL_TX_DUR_s) {
				static bool_t bClean = FALSE;

				bClean = bClean ? FALSE : TRUE;

				if (bClean) {
					// 交互にデータ投入、クリーンを繰り返す
					ToCoNet_MsgPl_bSetMessage(0, 0, 0, NULL); // clean slot
					ToCoNet_MsgPl_bSetMessage(1, 0, 8, (uint8*)"01234567");
					ToCoNet_MsgPl_bSetMessage(2, 0, 0, NULL);
					ToCoNet_MsgPl_bSetMessage(3, 0, 0, (uint8*)"abcdefgh");
					//V_PRINTF(LB"! MsgPl Clean");
				} else {
					uint8 au8pl[TOCONET_MOD_MESSAGE_POOL_MAX_MESSAGE];
						// メッセージプールの最大バイト数は 64 なので、これに収まる数とする。
					uint8 *q = au8pl;

					for (i = 0; i < ADDRKEYA_MAX_HISTORY; i++) {
						if (sEndDevList.au32ScanListAddr[i]) {

							uint16 u16Sec = (u32TickCount_ms
									- sEndDevList.au32ScanListTick[i]) / 1000;
							if (u16Sec >= 0xF0)
								continue; // 古すぎるので飛ばす

							S_OCTET(u16Sec & 0xFF);
							S_BE_DWORD(sEndDevList.au32ScanListAddr[i]);
						}
					}
					S_OCTET(0xFF);

					S_OCTET('A'); // ダミーデータ(不要：テスト目的)
					S_OCTET('B');
					S_OCTET('C');
					S_OCTET('D');

					while (q - au8pl < TOCONET_MOD_MESSAGE_POOL_MAX_MESSAGE) {
						S_OCTET('X');
					}

					ToCoNet_MsgPl_bSetMessage(0, 0, q - au8pl, au8pl);
					ToCoNet_MsgPl_bSetMessage(1, 0, 0, NULL);
					ToCoNet_MsgPl_bSetMessage(2, 0, q - au8pl, au8pl);
					ToCoNet_MsgPl_bSetMessage(3, 0, 0, NULL);

					//V_PRINTF(LB"! MsgPl Set");
				}

				u8Ct_s = 0;
			}
		} else {
			;
		}
		break;

	default:
		break;
	}
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
