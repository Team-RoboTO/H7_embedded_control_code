#ifndef TELEMETRY_H
#define TELEMETRY_H

/*
 * telemetry.h
 * -----------
 * Modulo unico per la telemetria STM32 -> ESP32.
 *
 * Formato pacchetto:
 *   Header (7 byte fissi):
 *     [SYNC 0xAA][SYNC 0x55][PKT_LEN 1B][TIMESTAMP 4B]
 *   Payload (PKT_LEN byte, TLV ripetuto):
 *     [TAG 1B][LEN 1B][VALUE LEN byte] ...
 *
 * Vincoli derivati dal formato (conseguenze dirette delle dimensioni
 * scelte per i campi header, non valori arbitrari):
 *   - PKT_LEN e' 1 byte  -> payload max 255 byte
 *   - pacchetto totale max = 7 + 255 = 262 byte
 *
 * Strategia di trasmissione: singolo buffer statico + flag di stato
 * ("busy"), invece di double buffering. Giustificazione numerica
 * (calcolo diretto dai parametri scelti, non una stima):
 *   - baud rate 921600, frame UART a 11 bit/byte (parita' abilitata:
 *     1 start + 8 dati + 1 parita' + 1 stop)
 *     -> capacita' canale = 921600 / 11 ~= 83 782 byte/s
 *   - caso d'uso: 6-8 campi float @ 1000 Hz
 *     -> pacchetto 43-55 byte -> tempo di trasmissione 0,51-0,66 ms
 *     -> utilizzo canale 51-66% su un ciclo di 1 ms, margine 34-49%
 *   - margine sufficiente per NON aver bisogno di double buffering
 *
 * ATTENZIONE: HAL_UART_TxCpltCallback() e' una funzione "weak" globale
 * della libreria HAL, condivisa da TUTTE le periferiche UART del progetto.
 * Se altrove nel tuo codice esiste gia' una definizione di questa funzione
 * (es. per un'altra UART), va unificata in un solo punto con un controllo
 * su huart->Instance, altrimenti il linker segnala simbolo duplicato.
 *
 * ADATTARE: questo modulo assume che la UART verso l'ESP32 sia huart2.
 * Se usi un'altra istanza (huart1, huart3...), cambia sia l'handle in
 * telemetry.c sia il controllo huart->Instance nella callback.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* --------------------- Costruzione pacchetto --------------------- */

#define TELEMETRY_SYNC_BYTE_1      0xAAu
#define TELEMETRY_SYNC_BYTE_2      0x55u
#define TELEMETRY_HEADER_SIZE      7u      /* 2 sync + 1 len + 4 timestamp */
#define TELEMETRY_MAX_PAYLOAD_SIZE 255u    /* limite imposto da PKT_LEN a 1 byte */
#define TELEMETRY_MAX_PACKET_SIZE  (TELEMETRY_HEADER_SIZE + TELEMETRY_MAX_PAYLOAD_SIZE)

/*
 * Un campo TLV cosi' come lo passa il chiamante.
 * NOTA: 'value' punta ai byte grezzi del dato (es. l'indirizzo di un float).
 * Il puntatore deve restare valido solo per la durata della chiamata a
 * build_telemetry_packet(), perche' i byte vengono copiati subito (memcpy),
 * non referenziati in modo differito.
 */
typedef struct {
    uint8_t         tag;
    uint8_t         len;
    const uint8_t  *value;
} TelemetryField;

/* Helper per costruire rapidamente un campo a partire da un float.
 * sizeof(float) e' garantito == 4 su Cortex-M (IEEE754 single precision).
 * Definita "static inline" qui nell'header: ogni file che la include ne
 * ottiene una propria copia privata (linkage "static"), espansa dal
 * compilatore direttamente nel punto di chiamata. */
static inline TelemetryField telemetry_field_float(uint8_t tag, const float *value)
{
    TelemetryField f;
    f.tag   = tag;
    f.len   = (uint8_t)sizeof(float);
    f.value = (const uint8_t *)value;
    return f;
}

/* Helper generico per un campo di larghezza arbitraria (es. uint16_t, uint8_t...) */
static inline TelemetryField telemetry_field_raw(uint8_t tag, const void *value, uint8_t len)
{
    TelemetryField f;
    f.tag   = tag;
    f.len   = len;
    f.value = (const uint8_t *)value;
    return f;
}

/*
 * Costruisce il pacchetto completo dentro out_buf.
 *
 * Ritorna il numero di byte scritti (= dimensione totale del pacchetto),
 * oppure 0 in caso di errore (payload troppo grande per il formato,
 * o buffer di destinazione troppo piccolo).
 */
size_t build_telemetry_packet(uint8_t *out_buf,
                               size_t out_buf_size,
                               uint32_t timestamp_ms,
                               const TelemetryField *fields,
                               uint8_t num_fields);

/* --------------------- Gestione trasmissione DMA --------------------- */

/* Inizializza lo stato interno del modulo (da chiamare una volta, in setup) */
void telemetry_tx_init(void);

/*
 * Costruisce e invia un pacchetto di telemetria via DMA, SE il buffer
 * e' libero (trasmissione precedente completata).
 *
 * Ritorna:
 *   true  - pacchetto costruito e trasmissione DMA avviata
 *   false - saltato (DMA ancora occupato dal pacchetto precedente,
 *           oppure errore di costruzione pacchetto)
 *
 * NOTA: il ritorno "false" per busy non dovrebbe MAI verificarsi nel
 * caso d'uso dimensionato sopra (6-8 campi @ 1kHz). Se lo vedi accadere
 * spesso in pratica, e' un segnale che qualcosa nel sistema sta
 * introducendo piu' latenza del previsto e va indagato.
 */
bool telemetry_tx_send(uint32_t timestamp_ms,
                        const TelemetryField *fields,
                        uint8_t num_fields);

#endif /* TELEMETRY_H */