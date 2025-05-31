#include <xc.h>
#include <stdbool.h>
#include <string.h>

#include "../platform.h"

// Functions "exported" by this file
void platform_usart_pms_init(void);
void platform_usart_esp_init(void);
void platform_usart_tick_handler(const platform_timespec_t *tick);

/////////////////////////////////////////////////////////////////////////////

/**
 * State variables for UART
 * 
 * Separate contexts for PMS7003 (SERCOM0) and Laptop (SERCOM3)
 */
typedef struct ctx_usart_type {
    /// Pointer to the underlying register set
    sercom_usart_int_registers_t *regs;

    /// State variables for the transmitter
    struct {
        volatile platform_usart_tx_bufdesc_t *desc;
        volatile uint16_t nr_desc;
        volatile const char *buf;
        volatile uint16_t len;
    } tx;

    /// State variables for the receiver
    struct {
        volatile platform_usart_rx_async_desc_t *volatile desc;
        volatile platform_timespec_t ts_idle;
        volatile uint16_t idx;
    } rx;

    /// Configuration items
    struct {
        platform_timespec_t ts_idle_timeout;
    } cfg;

} ctx_usart_t;

static ctx_usart_t ctx_uart_esp;    // Context for ESP8266  (SERCOM0)
static ctx_usart_t ctx_uart_co2;    // Context for MH-Z19C  (SERCOM1)
static ctx_usart_t ctx_uart_pms;    // Context for PMS5003T (SERCOM3)
static ctx_usart_t ctx_uart_gps;    // Context for NEO-6M   (SERCOM5)

// Configure ESP8266 UART (SERCOM0)
void platform_usart_esp_init(void) {
    #define UART0_REGS (&(SERCOM0_REGS->USART_INT))
    
    MCLK_REGS->MCLK_APBCMASK |= (1 << 1);
    GCLK_REGS->GCLK_PCHCTRL[17] = 0x00000042;
    while ((GCLK_REGS->GCLK_PCHCTRL[17] & 0x00000040) == 0) asm("nop");

    memset(&ctx_uart_esp, 0, sizeof(ctx_uart_esp));
    ctx_uart_esp.regs = UART0_REGS;

    UART0_REGS->SERCOM_CTRLA = 0x01;
    while ((UART0_REGS->SERCOM_SYNCBUSY & 0x01) != 0) asm("nop");
    UART0_REGS->SERCOM_CTRLA = (uint32_t)(0x4);
    
    UART0_REGS->SERCOM_CTRLA |= (0 << 16) | (1 << 20) | (0 << 24) | (1 << 30);
    UART0_REGS->SERCOM_BAUD = 63019;
    
    ctx_uart_esp.cfg.ts_idle_timeout.nr_sec = 0;
    ctx_uart_esp.cfg.ts_idle_timeout.nr_nsec = 468750;
    
    UART0_REGS->SERCOM_CTRLB |= (1 << 16) | (1 << 17);
    while ((UART0_REGS->SERCOM_SYNCBUSY & (1<<2)) != 0) asm("nop");
    
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 4);
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[4] = 0x1;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[2] = 0x03;

    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 5);
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[5] = 0x3;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[2] |= (0x03 << 4);
    
    UART0_REGS->SERCOM_CTRLA |= (1<<1);
    while ((UART0_REGS->SERCOM_SYNCBUSY & (1 << 1)) != 0) asm("nop");
    return;
}

//// Configure PMS7003 UART (SERCOM0)
//void platform_usart_pms_init(void) {
//    #define UART_REGS (&(SERCOM0_REGS->USART_INT))
//    
//    MCLK_REGS->MCLK_APBCMASK |= (1 << 1);
//    GCLK_REGS->GCLK_PCHCTRL[17] = 0x00000042;
//    while ((GCLK_REGS->GCLK_PCHCTRL[17] & 0x00000040) == 0) asm("nop");
//
//    memset(&ctx_uart_pms, 0, sizeof(ctx_uart_pms));
//    ctx_uart_pms.regs = UART_REGS;
//
//    UART_REGS->SERCOM_CTRLA = 0x01;
//    while ((UART_REGS->SERCOM_SYNCBUSY & 0x01) != 0) asm("nop");
//    UART_REGS->SERCOM_CTRLA = (uint32_t)(0x4);
//    
//    UART_REGS->SERCOM_CTRLA |= (1 << 20) | (0 << 24) | (1 << 30);
//    UART_REGS->SERCOM_BAUD = 63019;
//    
//    ctx_uart_pms.cfg.ts_idle_timeout.nr_sec = 0;
//    ctx_uart_pms.cfg.ts_idle_timeout.nr_nsec = 10648750;
//    
//    UART_REGS->SERCOM_CTRLB |= (0 << 0) | (0 << 6) | (1 << 16) | (1 << 17) | (1 << 22) | (1 << 23);
//    while ((UART_REGS->SERCOM_SYNCBUSY & (1<<2)) != 0) asm("nop");
//    
//    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 4);
//    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[4] = 0x1;
//    PORT_SEC_REGS->GROUP[0].PORT_PMUX[2] = 0x33;
//    
//    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 5);
//    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[5] = 0x3;
//    
//    UART_REGS->SERCOM_CTRLA |= (1<<1);
//    while ((UART_REGS->SERCOM_SYNCBUSY & (1 << 1)) != 0) asm("nop");
//    return;
//}

// Configure MH-Z19C UART (SERCOM1)
void platform_usart_co2_init(void) {
    #define UART1_REGS (&(SERCOM1_REGS->USART_INT))
    
    MCLK_REGS->MCLK_APBCMASK |= (1 << 2);
    GCLK_REGS->GCLK_PCHCTRL[18] = 0x00000042;
    while ((GCLK_REGS->GCLK_PCHCTRL[18] & 0x00000040) == 0) asm("nop");
    
    // Initialize the peripheral's context structure
    memset(&ctx_uart_co2, 0, sizeof(ctx_uart_co2));
    ctx_uart_co2.regs = UART1_REGS;
    
    UART1_REGS->SERCOM_CTRLA = 0x01;
    while ((UART1_REGS->SERCOM_SYNCBUSY & 0x01) != 0) asm("nop");
    UART1_REGS->SERCOM_CTRLA = (uint32_t)(0x4);
    
    UART1_REGS->SERCOM_CTRLA |= (0 << 16) | (1 << 20) | (0 << 24) | (1 << 30);
    UART1_REGS->SERCOM_BAUD = 63019;
    
    ctx_uart_co2.cfg.ts_idle_timeout.nr_sec = 0;
    ctx_uart_co2.cfg.ts_idle_timeout.nr_nsec = 468750;
    
    UART1_REGS->SERCOM_CTRLB |= (1 << 16) | (1 << 17) | (1 << 22) | (1 << 23);
    while ((UART1_REGS->SERCOM_SYNCBUSY & (1 << 2)) != 0) asm("nop");
    
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 16);
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[16] = 0x1;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[8] = 0x02;
    
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 17);
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[17] = 0x3;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[8] |= (0x02 << 4);
    
    UART1_REGS->SERCOM_CTRLA |= (1 << 1);
    while ((UART1_REGS->SERCOM_SYNCBUSY & (1 << 1)) != 0) asm("nop");
    return;
}

// Configure PMS5003T UART (SERCOM3)
void platform_usart_pms_init(void) {
    #define UART3_REGS (&(SERCOM3_REGS->USART_INT))
    
    MCLK_REGS->MCLK_APBCMASK |= (1 << 4);
    GCLK_REGS->GCLK_PCHCTRL[20] = 0x00000042;
    while ((GCLK_REGS->GCLK_PCHCTRL[20] & 0x00000040) == 0) asm("nop");

    memset(&ctx_uart_pms, 0, sizeof(ctx_uart_pms));
    ctx_uart_esp.regs = UART3_REGS;

    UART3_REGS->SERCOM_CTRLA = 0x01;
    while ((UART3_REGS->SERCOM_SYNCBUSY & 0x01) != 0) asm("nop");
    UART3_REGS->SERCOM_CTRLA = (uint32_t)(0x4);
    
    UART3_REGS->SERCOM_CTRLA |= (0 << 16) | (0 << 20) | (0 << 24) | (1 << 30);
    UART3_REGS->SERCOM_BAUD = 63019;
    
    ctx_uart_pms.cfg.ts_idle_timeout.nr_sec = 0;
    ctx_uart_pms.cfg.ts_idle_timeout.nr_nsec = 468750;
    
    UART3_REGS->SERCOM_CTRLB |= (0 << 16) | (1 << 17) | (1 << 22) | (1 << 23);
    while ((UART3_REGS->SERCOM_SYNCBUSY & (1 << 2)) != 0) asm("nop");
    
    PORT_SEC_REGS->GROUP[1].PORT_DIRCLR = (1 << 2);
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[2] = 0x3;
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[1] |= 0x02;
    
    UART3_REGS->SERCOM_CTRLA |= (1 << 1);
    while ((UART3_REGS->SERCOM_SYNCBUSY & (1 << 1)) != 0) asm("nop");
    return;
}

//// Configure Laptop UART (SERCOM3)
//void platform_usart_esp_init(void) {
//    #define UART_REGS (&(SERCOM3_REGS->USART_INT))
//    
//    MCLK_REGS->MCLK_APBCMASK |= (1 << 4);
//    GCLK_REGS->GCLK_PCHCTRL[20] = 0x00000042;
//    while ((GCLK_REGS->GCLK_PCHCTRL[20] & 0x00000040) == 0) asm("nop");
//    
//    memset(&ctx_uart_esp, 0, sizeof(ctx_uart_esp));
//    ctx_uart_esp.regs = UART_REGS;
//    
//    UART_REGS->SERCOM_CTRLA = 0x01;
//    while ((UART_REGS->SERCOM_SYNCBUSY & 0x01) != 0) asm("nop");
//    UART_REGS->SERCOM_CTRLA = (uint32_t)(0x4);
//    
//    UART_REGS->SERCOM_CTRLA |= (0 << 16) | (1 << 20) | (0 << 24) | (1 << 30);
//    UART_REGS->SERCOM_BAUD = 63019;
//    
//    ctx_uart_esp.cfg.ts_idle_timeout.nr_sec = 0;
//    ctx_uart_esp.cfg.ts_idle_timeout.nr_nsec = 468750;
//    
//    UART_REGS->SERCOM_CTRLB |= (1 << 16) | (1 << 17) | (0b11 << 22);
//    while ((UART_REGS->SERCOM_SYNCBUSY & (1<<2)) != 0) asm("nop");
//    
//    PORT_SEC_REGS->GROUP[1].PORT_DIRSET = (1 << 8);
//    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[8] = 0x1;
//    PORT_SEC_REGS->GROUP[1].PORT_PMUX[4] = 0x33;
//    
//    PORT_SEC_REGS->GROUP[1].PORT_DIRCLR = (1 << 9);
//    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[9] = 0x3;
//    
//    UART_REGS->SERCOM_CTRLA |= (1<<1);
//    while ((UART_REGS->SERCOM_SYNCBUSY & (1<<1)) != 0) asm("nop");
//    return;
//}

// Configure NEO-6M UART (SERCOM5)
void platform_usart_gps_init(void) {
    #define UART5_REGS (&(SERCOM5_REGS->USART_INT))
    
    MCLK_REGS->MCLK_APBCMASK |= (1 << 6);
    GCLK_REGS->GCLK_PCHCTRL[22] = 0x00000042;
    while ((GCLK_REGS->GCLK_PCHCTRL[22] & 0x00000040) == 0) asm("nop");

    memset(&ctx_uart_gps, 0, sizeof(ctx_uart_gps));
    ctx_uart_gps.regs = UART5_REGS;

    UART5_REGS->SERCOM_CTRLA = 0x01;
    while ((UART5_REGS->SERCOM_SYNCBUSY & 0x01) != 0) asm("nop");
    UART5_REGS->SERCOM_CTRLA = (uint32_t)(0x4);
    
    UART5_REGS->SERCOM_CTRLA |= (0 << 16) | (1 << 20) | (0 << 24) | (1 << 30);
    UART5_REGS->SERCOM_BAUD = 63019;
    
    ctx_uart_gps.cfg.ts_idle_timeout.nr_sec = 0;
    ctx_uart_gps.cfg.ts_idle_timeout.nr_nsec = 468750;
    
    UART5_REGS->SERCOM_CTRLB |= (0 << 16) | (1 << 17) | (1 << 22) | (1 << 23);
    while ((UART5_REGS->SERCOM_SYNCBUSY & (1 << 2)) != 0) asm("nop");
    
    PORT_SEC_REGS->GROUP[1].PORT_DIRCLR = (1 << 3);
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[3] = 0x3;
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[1] |= (0x03 << 4);
    
    UART5_REGS->SERCOM_CTRLA |= (1 << 1);
    while ((UART5_REGS->SERCOM_SYNCBUSY & (1 << 1)) != 0) asm("nop");
    return;
}

// Helper abort routine for USART reception
static void usart_rx_abort_helper(ctx_usart_t *ctx)
{
    if (ctx->rx.desc != NULL) {
        ctx->rx.desc->compl_type = PLATFORM_USART_RX_COMPL_DATA;
        ctx->rx.desc->compl_info.data_len = ctx->rx.idx;
        ctx->rx.desc = NULL;
    }
    ctx->rx.ts_idle.nr_sec = 0;
    ctx->rx.ts_idle.nr_nsec = 0;
    ctx->rx.idx = 0;
    return;
}

static void usart_tick_handler_common(
    ctx_usart_t *ctx, const platform_timespec_t *tick)
{
    uint16_t status = 0x0000;
    uint8_t data = 0x00;
    platform_timespec_t ts_delta;

    // TX handling
    if ((ctx->regs->SERCOM_INTFLAG & (1 << 0)) != 0) {
        if (ctx->tx.len > 0) {
            /*
             * There is still something to transmit in the working
             * copy of the current descriptor.
             */
            ctx->regs->SERCOM_DATA = *(ctx->tx.buf++);
            --ctx->tx.len;
        }
        if (ctx->tx.len == 0) {
            // Load a new descriptor
            ctx->tx.buf = NULL;
            if (ctx->tx.nr_desc > 0) {
                /*
                 * There's at least one descriptor left to
                 * transmit
                 * 
                 * If either ->buf or ->len of the candidate
                 * descriptor refer to an empty buffer, the
                 * next invocation of this routine will cause
                 * the next descriptor to be evaluated.
                 */
                ctx->tx.buf = ctx->tx.desc->buf;
                ctx->tx.len = ctx->tx.desc->len;

                ++ctx->tx.desc;
                --ctx->tx.nr_desc;

                if (ctx->tx.buf == NULL || ctx->tx.len == 0) {
                    ctx->tx.buf = NULL;
                    ctx->tx.len = 0;
                }
            } else {
                /*
                 * No more descriptors available
                 * 
                 * Clean up the corresponding context data so
                 * that we don't trip over them on the next
                 * invocation.
                 */
                ctx->regs->SERCOM_INTENCLR = 0x01;
                ctx->tx.desc = NULL;
                ctx->tx.buf = NULL;
            }
        }
    }
    
    // RX handling
    if ((ctx->regs->SERCOM_INTFLAG & (1 << 2)) != 0) {
        /*
         * There are unread data
         * 
         * To enable readout of error conditions, STATUS must be read
         * before reading DATA.
         * 
         * NOTE: Piggyback on Bit 15, as it is undefined for this
         *       platform.
         */
        status = ctx->regs->SERCOM_STATUS | 0x8000;
        data = (uint8_t)(ctx->regs->SERCOM_DATA);
    }
    
    do {
        if (ctx->rx.desc == NULL) {
            break;
        }

        if ((status & 0x8003) == 0x8000) {
            ctx->rx.desc->buf[ctx->rx.idx++] = data;
            ctx->rx.ts_idle = *tick;
        }
        ctx->regs->SERCOM_STATUS |= (status & 0x00F7);

        // Some housekeeping
        if (ctx->rx.idx >= ctx->rx.desc->max_len) {
            // Buffer completely filled
            usart_rx_abort_helper(ctx);
            break;
        } else if (ctx->rx.idx > 0) {
            platform_tick_delta(&ts_delta, tick, &ctx->rx.ts_idle);
            if (platform_timespec_compare(&ts_delta, &ctx->cfg.ts_idle_timeout) >= 0) {
                // IDLE timeout
                usart_rx_abort_helper(ctx);
                break;
            }
        }
    } while (0);

    // Done
    return;
}
void platform_usart_tick_handler(const platform_timespec_t *tick)
{
    usart_tick_handler_common(&ctx_uart_esp, tick);
    usart_tick_handler_common(&ctx_uart_co2, tick);
    usart_tick_handler_common(&ctx_uart_pms, tick);
    usart_tick_handler_common(&ctx_uart_gps, tick);
}

/// Maximum number of bytes that may be sent (or received) in one transaction
#define NR_USART_CHARS_MAX (65528)

/// Maximum number of fragments for USART TX
#define NR_USART_TX_FRAG_MAX (32)

// Enqueue a buffer for transmission
static bool usart_tx_busy(ctx_usart_t *ctx)
{
    return (ctx->tx.len > 0) || (ctx->tx.nr_desc > 0) ||
        ((ctx->regs->SERCOM_INTFLAG & (1 << 0)) == 0);
}
static bool usart_tx_async(ctx_usart_t *ctx,
    const platform_usart_tx_bufdesc_t *desc,
    unsigned int nr_desc)
{
    uint16_t avail = NR_USART_CHARS_MAX;
    unsigned int x, y;

    if (!desc || nr_desc == 0)
        return true;
    else if (nr_desc > NR_USART_TX_FRAG_MAX)
        return false;

    if (usart_tx_busy(ctx))
        return false;

    for (x = 0, y = 0; x < nr_desc; ++x) {
        if (desc[x].len > avail) {
            // IF the message is too long, don't enqueue.
            return false;
        }

        avail -= desc[x].len;
        ++y;
    }

    // The tick will trigger the transfer
    ctx->tx.desc = desc;
    ctx->tx.nr_desc = nr_desc;
    return true;
}
static void usart_tx_abort(ctx_usart_t *ctx)
{
    ctx->tx.nr_desc = 0;
    ctx->tx.desc = NULL;
    ctx->tx.len = 0;
    ctx->tx.buf = NULL;
    return;
}

bool platform_usart_esp_tx_async(
    const platform_usart_tx_bufdesc_t *desc,
    unsigned int nr_desc)
{
    return usart_tx_async(&ctx_uart_esp, desc, nr_desc);
}
bool platform_usart_esp_tx_busy(void)
{
    return usart_tx_busy(&ctx_uart_esp);
}
void platform_usart_esp_tx_abort(void)
{
    usart_tx_abort(&ctx_uart_esp);
    return;
}

bool platform_usart_co2_tx_async(
    const platform_usart_tx_bufdesc_t *desc,
    unsigned int nr_desc)
{
    return usart_tx_async(&ctx_uart_co2, desc, nr_desc);
}
bool platform_usart_co2_tx_busy(void)
{
    return usart_tx_busy(&ctx_uart_co2);
}
void platform_usart_co2_tx_abort(void)
{
    usart_tx_abort(&ctx_uart_co2);
    return;
}

static bool usart_rx_busy(ctx_usart_t *ctx)
{
    return (ctx->rx.desc) != NULL;
}
static bool usart_rx_async(ctx_usart_t *ctx, platform_usart_rx_async_desc_t *desc)
{
    // Check some items first
    if (!desc|| !desc->buf || desc->max_len == 0 || desc->max_len > NR_USART_CHARS_MAX)
        return false;

    // Invalid descriptor
    if ((ctx->rx.desc) != NULL)
        // Don't clobber an existing buffer
        return false;

    desc->compl_type = PLATFORM_USART_RX_COMPL_NONE;
    desc->compl_info.data_len = 0;
    ctx->rx.idx = 0;
    platform_tick_hrcount(&ctx->rx.ts_idle);
    ctx->rx.desc = desc;
    return true;
}

// API-visible items
bool platform_usart_esp_rx_async(platform_usart_rx_async_desc_t *desc)
{
    return usart_rx_async(&ctx_uart_esp, desc);
}
bool platform_usart_esp_rx_busy(void)
{
    return usart_rx_busy(&ctx_uart_esp);
}
void platform_usart_esp_rx_abort(void)
{
    usart_rx_abort_helper(&ctx_uart_esp);
}

bool platform_usart_co2_rx_async(platform_usart_rx_async_desc_t *desc)
{
    return usart_rx_async(&ctx_uart_co2, desc);
}
bool platform_usart_co2_rx_busy(void)
{
    return usart_rx_busy(&ctx_uart_co2);
}
void platform_usart_co2_rx_abort(void)
{
    usart_rx_abort_helper(&ctx_uart_co2);
}

bool platform_usart_pms_rx_async(platform_usart_rx_async_desc_t *desc)
{
    return usart_rx_async(&ctx_uart_pms, desc);
}
bool platform_usart_pms_rx_busy(void)
{
    return usart_rx_busy(&ctx_uart_pms);
}
void platform_usart_pms_rx_abort(void)
{
    usart_rx_abort_helper(&ctx_uart_pms);
}

bool platform_usart_gps_rx_async(platform_usart_rx_async_desc_t *desc)
{
    return usart_rx_async(&ctx_uart_gps, desc);
}
bool platform_usart_gps_rx_busy(void)
{
    return usart_rx_busy(&ctx_uart_gps);
}
void platform_usart_gps_rx_abort(void)
{
    usart_rx_abort_helper(&ctx_uart_gps);
}