/* Exercise production CAN filters and FIFO draining; only HAL is simulated. */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "../bsp/can/bsp_can.c"

CAN_HandleTypeDef hcan1, hcan2;
static CAN_FilterTypeDef configured[3];
static unsigned filter_count, starts, notifications;
static CAN_RxHeaderTypeDef queued[8];
static uint8_t payload[8];
static unsigned queued_count, read_count, callback_count;
static uint8_t received[8];
static char heap[4096] __attribute__((aligned(8)));
static ptrdiff_t heap_used;

void *_sbrk(ptrdiff_t increment)
{
    if (increment < 0 || heap_used + increment > sizeof(heap)) return (void *)-1;
    void *result = heap + heap_used;
    heap_used += increment;
    return result;
}

static void host_call(uint32_t operation, const void *argument)
{
    register uint32_t op __asm__("r0") = operation;
    register const void *arg __asm__("r1") = argument;
    __asm__ volatile("bkpt 0xab" : "+r"(op) : "r"(arg) : "memory");
}

static void finish(uint32_t code)
{
    uint32_t args[2] = {0x20026, code};
    host_call(0x20, args);
    for (;;) {}
}
#define STR_(n) #n
#define STR(n) STR_(n)
#define CHECK(condition) do { if (!(condition)) { \
    host_call(4, "FAIL line " STR(__LINE__) ": " #condition "\n"); finish(__LINE__); } } while (0)

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can) { starts++; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t flags)
{ notifications++; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *filter)
{
    CHECK(filter_count < 3);
    configured[filter_count++] = *filter;
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *can, uint32_t fifo)
{ return queued_count - read_count; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t data[])
{
    CHECK(read_count < queued_count);
    *header = queued[read_count];
    memset(data, payload[read_count++], 8);
    return HAL_OK;
}
static void callback(CANInstance *instance)
{
    CHECK(instance->rx_len == 8);
    received[callback_count++] = instance->rx_buff[0];
}
static void enqueue(uint32_t can_id, uint32_t ide, uint32_t rtr, uint32_t dlc, uint8_t value)
{
    CHECK(queued_count < 8);
    queued[queued_count] = (CAN_RxHeaderTypeDef){.StdId = can_id, .IDE = ide, .RTR = rtr, .DLC = dlc};
    payload[queued_count++] = value;
}

void test_main(void)
{
    CAN_Init_Config_s config = {.can_handle = &hcan1, .tx_id = 1,
        .rx_id = 0x201, .can_module_callback = callback};
    CANInstance *drive = CANRegister(&config);
    config.tx_id = 0x311; config.rx_id = 0x312;
    CANInstance *link = CANRegister(&config);
    config.can_handle = &hcan2; config.tx_id = 0x061; config.rx_id = 0x052;
    CANRegister(&config);
    CHECK(starts == 2 && notifications == 4 && filter_count == 3);
    for (unsigned i = 0; i < 3; ++i)
    {
        uint32_t can_id = (uint32_t[]){0x201, 0x312, 0x052}[i] << 5;
        CHECK(configured[i].FilterIdLow == can_id);
        CHECK(configured[i].FilterIdHigh == can_id);
        CHECK(configured[i].FilterMaskIdLow == can_id);
        CHECK(configured[i].FilterMaskIdHigh == can_id);
        CHECK(configured[i].FilterMode == CAN_FILTERMODE_IDLIST);
        CHECK(configured[i].FilterScale == CAN_FILTERSCALE_16BIT);
        CHECK(configured[i].FilterActivation == CAN_FILTER_ENABLE);
        CHECK(configured[i].SlaveStartFilterBank == 14);
    }
    CHECK(configured[0].FilterBank == 0 && configured[1].FilterBank == 1);
    CHECK(configured[2].FilterBank == 14);
    CHECK(drive->txconf.StdId == 1 && link->txconf.StdId == 0x311);

    enqueue(0x201, CAN_ID_STD, CAN_RTR_DATA, 8, 11);
    enqueue(0x312, CAN_ID_STD, CAN_RTR_DATA, 8, 22);
    enqueue(0x599, CAN_ID_STD, CAN_RTR_DATA, 8, 99);
    enqueue(0x201, CAN_ID_EXT, CAN_RTR_DATA, 8, 99);
    enqueue(0x201, CAN_ID_STD, CAN_RTR_REMOTE, 8, 99);
    enqueue(0x201, CAN_ID_STD, CAN_RTR_DATA, 9, 99);
    enqueue(0x201, CAN_ID_STD, CAN_RTR_DATA, 8, 33);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    CHECK(read_count == queued_count && callback_count == 3);
    CHECK(received[0] == 11 && received[1] == 22 && received[2] == 33);
    host_call(4, "PASS: real BSP filters initialize all four slots; FIFO drains and rejects invalid frames\n");
    finish(0);
}
