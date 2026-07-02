/*
 * telemetry.c
 * -----------
 * Vedi telemetry.h per la spiegazione del formato pacchetto, della
 * strategia di trasmissione e delle note di adattamento (huart2,
 * HAL_UART_TxCpltCallback condivisa).
 */

#include "telemetry.h"
#include <string.h>
#include "usart.h"   /* per l'handle huart2, generato da STM32CubeMX */

/* --------------------- Costruzione pacchetto --------------------- */

size_t build_telemetry_packet(uint8_t *out_buf,
                               size_t out_buf_size,
                               uint32_t timestamp_ms,
                               const TelemetryField *fields,
                               uint8_t num_fields)
{
    /* --- Primo passaggio: calcolo della dimensione totale del payload ---
     * Ogni campo TLV occupa (1 byte TAG + 1 byte LEN + N byte VALUE).
     * Questo passaggio e' obbligatorio prima di scrivere qualsiasi cosa,
     * perche' PKT_LEN nell'header deve essere gia' noto quando lo scriviamo. */
    uint16_t payload_len = 0;
    for (uint8_t i = 0; i < num_fields; i++) {
        payload_len += (uint16_t)(2u + fields[i].len);
    }

    if (payload_len > TELEMETRY_MAX_PAYLOAD_SIZE) {
        return 0; /* il set di campi richiesto eccede il limite del formato (PKT_LEN a 1 byte) */
    }

    size_t total_size = TELEMETRY_HEADER_SIZE + payload_len;
    if (total_size > out_buf_size) {
        return 0; /* buffer del chiamante troppo piccolo per contenere il pacchetto */
    }

    /* --- Secondo passaggio: scrittura effettiva --- */
    size_t idx = 0;

    out_buf[idx++] = TELEMETRY_SYNC_BYTE_1;
    out_buf[idx++] = TELEMETRY_SYNC_BYTE_2;
    out_buf[idx++] = (uint8_t)payload_len;

    /* Timestamp copiato byte-per-byte cosi' com'e' in memoria: valido perche'
     * STM32 (Cortex-M) ed ESP32 sono entrambi little-endian, fatto architetturale
     * gia' verificato in precedenza, quindi non serve nessuna conversione. */
    memcpy(&out_buf[idx], &timestamp_ms, sizeof(timestamp_ms));
    idx += sizeof(timestamp_ms);

    for (uint8_t i = 0; i < num_fields; i++) {
        out_buf[idx++] = fields[i].tag;
        out_buf[idx++] = fields[i].len;
        memcpy(&out_buf[idx], fields[i].value, fields[i].len);
        idx += fields[i].len;
    }

    return idx; /* == total_size */
}

/* --------------------- Gestione trasmissione DMA --------------------- */

__attribute__((section(".AXI_SRAM"), aligned(32))) uint8_t packet_buf[TELEMETRY_MAX_PACKET_SIZE];
static volatile bool tx_busy = false;

void telemetry_tx_init(void)
{
    tx_busy = false;
}

bool telemetry_tx_send(uint32_t timestamp_ms,
                        const TelemetryField *fields,
                        uint8_t num_fields)
{
    if (tx_busy) {
        /* DMA ancora occupato con il pacchetto precedente: campione
         * saltato. Nel dimensionamento attuale (6-8 campi @ 1kHz)
         * questo non dovrebbe accadere in condizioni normali. */
        return false;
    }

    size_t packet_size = build_telemetry_packet(packet_buf, sizeof(packet_buf),
                                                  timestamp_ms, fields, num_fields);
    if (packet_size == 0) {
        return false; /* errore di costruzione (payload troppo grande o buffer insufficiente) */
    }

    tx_busy = true;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart10, packet_buf, packet_size);
    if (status != HAL_OK) {
        /* L'avvio della DMA e' fallito: libero subito il flag,
         * altrimenti il buffer resterebbe bloccato "occupato" per sempre. */
        tx_busy = false;
        return false;
    }

    return true;
}

/*
 * Callback HAL invocata automaticamente quando la DMA ha finito di
 * trasmettere. E' una funzione "weak" globale: se esiste gia' altrove
 * nel progetto, unificare qui il controllo su huart->Instance invece
 * di avere due definizioni (errore di linking altrimenti).
 */


/* ------------------- Esempio d'uso -------------------
 *
 * Nel setup:
 *   telemetry_tx_init();
 *
 * In un timer hardware a 1 kHz (es. HAL_TIM_PeriodElapsedCallback
 * di un TIM configurato a 1000 Hz), oppure nel loop principale se
 * gestisci il timing altrimenti:
 *
 *   float motor1_rpm = ...;
 *   float motor1_current = ...;
 *   float imu_ax = ...;
 *   // ... fino a 6-8 campi ...
 *
 *   TelemetryField fields[] = {
 *       telemetry_field_float(0x01, &motor1_rpm),
 *       telemetry_field_float(0x02, &motor1_current),
 *       telemetry_field_float(0x10, &imu_ax),
 *   };
 *
 *   telemetry_tx_send(HAL_GetTick(), fields, sizeof(fields)/sizeof(fields[0]));
 */