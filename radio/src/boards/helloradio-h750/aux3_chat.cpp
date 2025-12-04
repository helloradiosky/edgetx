#include "aux3_chat.h"

#if defined(MODULE_XIAOZHI_CHAT)

#include "stm32_gpio.h"
#include "board.h"
#include "debug.h"
#include "hal/serial_port.h"
#include "stm32_serial_driver.h"
#include "stm32_usart_driver.h"

#include <string.h>

namespace {

constexpr uint32_t V15_CHAT_UART_RX_BUFFER_SIZE = 512;
constexpr uint32_t V15_CHAT_UART_TX_BUFFER_SIZE = 256;

static const stm32_usart_t v15ChatUSART = {
  .USARTx = XIAOZHI_UART_USART,
  .txGPIO = XIAOZHI_UART_TX_GPIO,
  .rxGPIO = XIAOZHI_UART_RX_GPIO,
  .IRQn = XIAOZHI_UART_USART_IRQn,
  .IRQ_Prio = 7,
  .txDMA = XIAOZHI_UART_DMA_TX,
  .txDMA_Stream = XIAOZHI_UART_DMA_TX_STREAM,
  .txDMA_Channel = XIAOZHI_UART_DMA_TX_CHANNEL,
  .rxDMA = XIAOZHI_UART_DMA_RX,
  .rxDMA_Stream = XIAOZHI_UART_DMA_RX_STREAM,
  .rxDMA_Channel = XIAOZHI_UART_DMA_RX_CHANNEL,
  .set_input = nullptr,
  .txDMA_IRQn = static_cast<IRQn_Type>(-1),
  .txDMA_IRQ_Prio = 0,
};

DEFINE_STM32_SERIAL_PORT(V15Chat, v15ChatUSART,
                         V15_CHAT_UART_RX_BUFFER_SIZE,
                         V15_CHAT_UART_TX_BUFFER_SIZE);

static const etx_serial_port_t v15ChatPort = {
  .name = "V15-XiaoZhi-UART",
  .uart = &STM32SerialDriver,
  .hw_def = REF_STM32_SERIAL_PORT(V15Chat),
  .set_pwr = nullptr,
};

static void *s_chatCtx = nullptr;
static uint32_t s_baudrate = V15_CHAT_UART_DEFAULT_BAUDRATE;
static bool s_hasNewLine = false;
static char s_latestLine[V15_CHAT_UART_LINE_BUFFER_SIZE] = {0};
static char s_lineAssembly[V15_CHAT_UART_LINE_BUFFER_SIZE] = {0};
static uint32_t s_lineLen = 0;

static inline const etx_serial_driver_t *driver()
{
  return v15ChatPort.uart;
}

static inline bool isStateLine(const char* text)
{
  return text && (strstr(text, "STATE: ") || strstr(text, "State: ") ||
                  strstr(text, "state: ")) ||strstr(text, "WifiStation");
}

static inline void saveCompletedLine()
{
  if (s_lineLen == 0) {
    return;
  }

  s_lineAssembly[s_lineLen] = '\0';

  if (isStateLine(s_lineAssembly)) {
    strncpy(s_latestLine, s_lineAssembly, sizeof(s_latestLine) - 1);
    s_latestLine[sizeof(s_latestLine) - 1] = '\0';
    s_hasNewLine = true;
    //TRACE("V15 AUX3 RX state line: %s", s_latestLine);
  }

  s_lineLen = 0;
  s_lineAssembly[0] = '\0';
}

}  // namespace

void v15ChatUartInit(uint32_t baudrate)
{
  if (s_chatCtx) {
    if (baudrate != s_baudrate) {
      v15ChatUartSetBaudrate(baudrate);
    }
    return;
  }

  etx_serial_init params = {
    .baudrate = baudrate,
    .encoding = ETX_Encoding_8N1,
    .direction = ETX_Dir_TX_RX,
    .polarity = ETX_Pol_Normal,
  };

  s_chatCtx = driver()->init(v15ChatPort.hw_def, &params);
  if (s_chatCtx) {
    s_baudrate = baudrate;
    v15ChatUartClearRxBuffer();
    TRACE("V15 AUX3 UART ready @ %lu baud", static_cast<unsigned long>(baudrate));
  }
  else {
    TRACE("V15 AUX3 UART init failed");
  }
}

void v15ChatUartDeinit()
{
  if (!s_chatCtx) {
    return;
  }

  driver()->deinit(s_chatCtx);
  s_chatCtx = nullptr;
  s_hasNewLine = false;
  s_lineLen = 0;
  s_latestLine[0] = '\0';
  s_lineAssembly[0] = '\0';
}

bool v15ChatUartIsReady()
{
  return s_chatCtx != nullptr;
}

uint32_t v15ChatUartGetBaudrate()
{
  if (!s_chatCtx || !driver()->getBaudrate) {
    return s_baudrate;
  }

  return driver()->getBaudrate(s_chatCtx);
}

void v15ChatUartSetBaudrate(uint32_t baudrate)
{
  s_baudrate = baudrate;
  if (s_chatCtx && driver()->setBaudrate) {
    driver()->setBaudrate(s_chatCtx, baudrate);
  }
}

void v15ChatUartClearRxBuffer()
{
  if (s_chatCtx && driver()->clearRxBuffer) {
    driver()->clearRxBuffer(s_chatCtx);
  }
}

uint32_t v15ChatUartAvailable()
{
  if (!s_chatCtx || !driver()->getBufferedBytes) {
    return 0;
  }

  int count = driver()->getBufferedBytes(s_chatCtx);
  return (count > 0) ? static_cast<uint32_t>(count) : 0;
}

bool v15ChatUartReadByte(uint8_t *data)
{
  if (!s_chatCtx || !data || !driver()->getByte) {
    return false;
  }

  return driver()->getByte(s_chatCtx, data) > 0;
}

uint32_t v15ChatUartRead(uint8_t *data, uint32_t maxLen)
{
  if (!data || maxLen == 0) {
    return 0;
  }

  uint32_t count = 0;
  while (count < maxLen && v15ChatUartReadByte(&data[count])) {
    ++count;
  }
  return count;
}

void v15ChatUartSendByte(uint8_t byte)
{
  if (!s_chatCtx || !driver()->sendByte) {
    return;
  }

  driver()->sendByte(s_chatCtx, byte);
}

void v15ChatUartSendBuffer(const uint8_t *data, uint32_t len)
{
  if (!s_chatCtx || !data || len == 0) {
    return;
  }

  if (driver()->sendBuffer) {
    driver()->sendBuffer(s_chatCtx, data, len);
  }
  else if (driver()->sendByte) {
    for (uint32_t i = 0; i < len; ++i) {
      driver()->sendByte(s_chatCtx, data[i]);
    }
  }
}

void v15ChatUartSendString(const char *text)
{
  if (!text) {
    return;
  }

  v15ChatUartSendBuffer(reinterpret_cast<const uint8_t *>(text), strlen(text));
}

void v15ChatUartWaitTxCompleted()
{
  if (!s_chatCtx || !driver()->waitForTxCompleted) {
    return;
  }

  driver()->waitForTxCompleted(s_chatCtx);
}

bool v15ChatUartPollLine()
{
  enum ParseState {
    WaitLevel,
    WaitOpenParen,
    SkipParenContent,
    WaitPayloadStart,
    CollectPayload,
  };

  static ParseState parseState = WaitLevel;

  uint8_t byte = 0;
  bool updated = false;

  while (v15ChatUartReadByte(&byte)) {

    if (byte == '\r') {
      if (parseState == CollectPayload && s_lineLen > 0) {
        s_lineAssembly[s_lineLen] = '\0';
        TRACE("V15 AUX3 RX payload: %s", s_lineAssembly);
        saveCompletedLine();
        updated = s_hasNewLine || updated;
      }
      parseState = WaitLevel;
      s_lineLen = 0;
      s_lineAssembly[0] = '\0';
      break;
    }

    if (byte == '\n') {
      continue;
    }

    switch (parseState) {
      case WaitLevel:
        if (byte == 'W' || byte == 'I') {
          parseState = WaitOpenParen;
        }
        break;

      case WaitOpenParen:
        if (byte == ' ') {
          break;
        }
        if (byte == '(') {
          parseState = SkipParenContent;
          break;
        }
        parseState = (byte == 'W' || byte == 'I') ? WaitOpenParen : WaitLevel;
        break;

      case SkipParenContent:
        if (byte == ')') {
          parseState = WaitPayloadStart;
        }
        break;

      case WaitPayloadStart:
        if (byte == ' ') {
          break;
        }
        s_lineLen = 0;
        s_lineAssembly[0] = '\0';
        parseState = CollectPayload;
        if (s_lineLen < (sizeof(s_lineAssembly) - 1)) {
          s_lineAssembly[s_lineLen++] = static_cast<char>(byte);
        }
        else {
          s_lineAssembly[sizeof(s_lineAssembly) - 2] = static_cast<char>(byte);
          s_lineAssembly[sizeof(s_lineAssembly) - 1] = '\0';
          s_lineLen = sizeof(s_lineAssembly) - 1;
        }
        break;

      case CollectPayload:
        if (s_lineLen < (sizeof(s_lineAssembly) - 1)) {
          s_lineAssembly[s_lineLen++] = static_cast<char>(byte);
        }
        else {
          memmove(s_lineAssembly, s_lineAssembly + 1, sizeof(s_lineAssembly) - 2);
          s_lineAssembly[sizeof(s_lineAssembly) - 2] = static_cast<char>(byte);
          s_lineAssembly[sizeof(s_lineAssembly) - 1] = '\0';
          s_lineLen = sizeof(s_lineAssembly) - 1;
        }
        break;
    }
  }
  return updated;
}

bool v15ChatUartHasNewLine()
{
  return s_hasNewLine;
}

bool v15ChatUartFetchLatestLine(char *buffer, uint32_t bufferLen, bool clearUpdated)
{
  if (!buffer || bufferLen == 0 || s_latestLine[0] == '\0') {
    return false;
  }

  strncpy(buffer, s_latestLine, bufferLen - 1);
  buffer[bufferLen - 1] = '\0';

  if (clearUpdated) {
    s_hasNewLine = false;
  }

  return true;
}

#endif  // MODULE_XIAOZHI_CHAT

#if !defined(MODULE_XIAOZHI_CHAT)

void v15ChatUartInit(uint32_t baudrate)
{
  (void)baudrate;
}
void v15ChatUartDeinit() {}
bool v15ChatUartIsReady() { return false; }
uint32_t v15ChatUartGetBaudrate() { return 0; }
void v15ChatUartSetBaudrate(uint32_t baudrate)
{
  (void)baudrate;
}
void v15ChatUartClearRxBuffer() {}
uint32_t v15ChatUartAvailable() { return 0; }
bool v15ChatUartReadByte(uint8_t *data)
{
  (void)data;
  return false;
}

uint32_t v15ChatUartRead(uint8_t *data, uint32_t maxLen)
{
  (void)data;
  (void)maxLen;
  return 0;
}

void v15ChatUartSendByte(uint8_t byte)
{
  (void)byte;
}

void v15ChatUartSendBuffer(const uint8_t *data, uint32_t len)
{
  (void)data;
  (void)len;
}

void v15ChatUartSendString(const char *text)
{
  (void)text;
}
void v15ChatUartWaitTxCompleted() {}
bool v15ChatUartPollLine() { return false; }
bool v15ChatUartHasNewLine() { return false; }
bool v15ChatUartFetchLatestLine(char *buffer, uint32_t bufferLen, bool clearUpdated)
{
  (void)buffer;
  (void)bufferLen;
  (void)clearUpdated;
  return false;
}

#endif
