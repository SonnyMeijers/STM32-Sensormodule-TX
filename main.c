/**
 ******************************************************************************
 * @file    main.c
 * @brief   Ultra-Low-Power Ethanol Sensor – STM32L031F6P6TR (TSSOP20)
 *
 * @details Volledig event-driven firmware voor een energiezuinig alcohol-
 *          sensorsysteem met SPEC 3SP-Ethanol-1000 elektrochemische sensor,
 *          transimpedantieversterker (TIA), NTC-temperatuurcompensatie en een
 *          discrete analoge RF-zender (MAX2661 mixer + BSS138 ASK/OOK modulatie).
 *          Ontworpen volgens de strikte ontwerpeisen uit het dictaat
 *          "Systematisch Ontwerpen van Energiezuinige Draadloze Sensorsystemen"
 *          (Dr. Ing. M.J. van der Horst).
 *
 *          KERNPRINCIPES TOEGEPAST:
 *          ─ Event-driven state machine (Stop Mode 0.35 µA tussen metingen)
 *          ─ Integer-only rekenkunde: eliminatie van software floating-point op Cortex-M0+
 *          ─ ADC Auto-Off + Auto-Wait + 8-bit resolutie (minimaliseer conversie-energie)
 *          ─ Alle ongebruikte GPIO-pinnen op Analog (geen zwevende CMOS Schmitt-triggers)
 *          ─ LPTIM1 op LSI (37 kHz) als autonome wektimer in Stop Mode (1 Hz)
 *          ─ Time-multiplexing van PA2 tussen ADC_IN2 (NTC) en USART2_TX (RF-zender)
 *          ─ Custom RF-protocol (4× 0xAA Preamble + 0xFF Sync + beknopte ASCII payload)
 *
 *          HARDWARE- & SENSOREIGENSCHAPPEN:
 *          ─ MCU: STM32L031F6P6TR (TSSOP20, ARM Cortex-M0+, max 32 MHz)
 *          ─ Sensor: SPEC 3SP-Ethanol-1000 (Package 110-202), 0–1000 ppm
 *          ─ TIA Versterkertrap: Terugkoppelweerstand Rf = 150 kΩ
 *          ─ Voeding / ADC Vref: 3.3V (3300 mV), 1 LSB = 3300 / 255 = 12.94 mV
 *          ─ Virtuele aarde (V_bias): ~94 mV (automatisch genuld via auto-zeroing)
 *          ─ RF-Zender: MAX2661 mixer + BSS138 MOSFETs voor On-Off Keying (ASK/OOK)
 *          ─ RF-Baudrate: 2400 baud (geoptimaliseerd voor analoge detector SNR)
 *
 *          PIN-TOEWIJZING (TSSOP20):
 *          ─ PA1: ADC_IN1  → SPEC Ethanol TIA signaaluitgang
 *          ─ PA2: ADC_IN2 / USART2_TX → NTC thermistor / RF ASK Modulatie-ingang
 *                 (time-multiplexed: Analog tijdens ADC, AF4 tijdens RF TX)
 *          ─ Overige pinnen: Analog Mode (geen pull)
 *
 *          VERMOGENSBUDGET (3.3V voeding, 2400 baud RF-transmissie):
 *          ─ Stop Mode:  ~0.35 µA × 958 ms  = 0.335 µA·s per cyclus
 *          ─ Actief:     ~1.5 mA  × 42 ms   = 63.0  µA·s per cyclus (~10 bytes @ 2400 baud)
 *          ─ Gemiddeld:  ~63.3 µA → P_gem = 209 µW (0.21 mW)  ≪  1 mW budget ✓
 *
 ******************************************************************************
 */

/* ═══════════════════════════════════════════════════════════════════════════
 * INCLUDES
 * ═══════════════════════════════════════════════════════════════════════════ */
#include "stm32l0xx_hal.h"
#include <stdio.h>      /* snprintf (alleen integer-formaten %u → geen float printf) */
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * COMPILE-TIME CONFIGURATIE & HARDWARECONSTANTS
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief  Gevoeligheid van de SPEC 3SP-Ethanol-1000 sensor in nA per ppm.
 *          Nominaal: 15 nA/ppm. Productietolerantie volgens datasheet: 9 tot 21 nA/ppm.
 *          Aanpasbaar per specifieke sensorkalibratiecode op de verpakking. */
#define SENSOR_SENSITIVITY_NA_PPM 15U  /* Nominaal 15 nA/ppm, aanpasbaar tot 21 */

/** @brief  Aantal ADC-samples voor auto-zeroing kalibratie van de virtuele aarde (~94 mV).
 *          MOET een macht van 2 zijn voor efficiënte bit-shift deling. */
#define CALIBRATION_SAMPLES       8U
#define CALIBRATION_SHIFT         3U    /**< log2(8) = 3 → deling via >>3 */

/** @brief  LPTIM1 AutoReload waarde voor ~1 Hz met LSI op ~37 kHz.
 *          Periode = (36999 + 1) / 37000 = 1.000 s (nominaal).
 *          Let op: LSI-tolerantie is ±30%, autonome wekbron in Stop Mode. */
#define LPTIM_ARR_1HZ             36999U

/** @brief  Aantal entries in de NTC-temperatuur lookup-tabel.
 *          Index 0 = 0°C, Index 40 = 40°C (inclusief) */
#define NTC_LUT_SIZE              41U

/** @brief  ADC-kanaaltoewijzing (STM32L031: kanaal = vaste pin-mapping) */
#define ADC_CH_ALCOHOL            ADC_CHANNEL_1   /**< PA1: SPEC TIA uitgang */
#define ADC_CH_NTC                ADC_CHANNEL_2   /**< PA2: NTC thermistor  */

/** @brief  RF Link Protocol constanten voor analoge OOK/ASK ontvanger */
#define RF_PREAMBLE_BYTE          0xAAU  /**< 0b10101010 voor bit-sync & AGC drempelvorming */
#define RF_PREAMBLE_COUNT         4U     /**< 4 bytes preamble (16.7 ms @ 2400 baud)        */
#define RF_SYNC_BYTE              0xFFU  /**< Start Frame Delimiter                         */

/** @brief  Zet op 1 voor productiefase. Schakelt SWD-pinnen (PA13/PA14) uit
 *          naar Analog mode om ~0.5 µA lekstroom te besparen. Zet op 0 tijdens debug. */
#define PRODUCTION_MODE           0

/* ═══════════════════════════════════════════════════════════════════════════
 * COMPILE-TIME VERIFICATIE (statische asserties, C11)
 * ═══════════════════════════════════════════════════════════════════════════ */
_Static_assert((1U << CALIBRATION_SHIFT) == CALIBRATION_SAMPLES,
               "CALIBRATION_SHIFT moet gelijk zijn aan log2(CALIBRATION_SAMPLES)");

_Static_assert(NTC_LUT_SIZE == 41U,
               "NTC_LUT_SIZE moet exact 41 zijn (0 t/m 40 graden Celsius)");

_Static_assert(SENSOR_SENSITIVITY_NA_PPM > 0U && SENSOR_SENSITIVITY_NA_PPM <= 30U,
               "SENSOR_SENSITIVITY_NA_PPM moet binnen realistisch bereik liggen (9-21 nA/ppm)");

/* ═══════════════════════════════════════════════════════════════════════════
 * NTC LOOKUP TABLE  (const → opgeslagen in Flash, niet in RAM)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * ONTWERPKEUZE: Een voorberekende LUT elimineert runtime floating-point
 * berekeningen voor de Steinhart-Hart / B-parameter vergelijking. Flash-
 * reads zijn "gratis" qua dynamisch vermogen vergeleken met het berekenen
 * van exp() in software op een Cortex-M0+.
 * Ref: Dictaat §3.3 & §3.5 "Software & Vermijden van dure rekenoperaties"
 *
 * CIRCUIT:
 *    VDD (3.3V) ─── [R_series  1MΩ] ──┬── ADC-ingang (PA2)
 *                                      │
 *                                  [NTC  1MΩ]
 *                                      │
 *                                     GND
 *
 * NTC-parameters: R_25 = 1 MΩ, B = 4661 K
 * V_adc = 3.3V × R_ntc / (R_ntc + 1MΩ)
 * ADC_8bit = round(255 × R_ntc / (R_ntc + 1MΩ))
 *
 * Hogere temperatuur → lagere R_ntc → lagere ADC-waarde.
 * De tabel is MONOTOON DALEND.
 */
static const uint8_t ntc_lut[NTC_LUT_SIZE] = {
    /*  [idx] °C : geschatte 8-bit ADC waarde                              */
    /*  0°C */ 206, /*  1 */ 203, /*  2 */ 201, /*  3 */ 198, /*  4 */ 195,
    /*  5°C */ 192, /*  6 */ 189, /*  7 */ 186, /*  8 */ 183, /*  9 */ 178,
    /* 10°C */ 173, /* 11 */ 169, /* 12 */ 165, /* 13 */ 161, /* 14 */ 157,
    /* 15°C */ 153, /* 16 */ 150, /* 17 */ 148, /* 18 */ 146, /* 19 */ 145,
    /* 20°C */ 144, /* 21 */ 141, /* 22 */ 137, /* 23 */ 134, /* 24 */ 131,
    /* 25°C */ 128, /* 26 */ 124, /* 27 */ 121, /* 28 */ 118, /* 29 */ 115,
    /* 30°C */ 111, /* 31 */ 108, /* 32 */ 104, /* 33 */ 101, /* 34 */  98,
    /* 35°C */  95, /* 36 */  92, /* 37 */  90, /* 38 */  87, /* 39 */  85,
    /* 40°C */  82
};

/* ═══════════════════════════════════════════════════════════════════════════
 * GLOBALE VARIABELEN
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief  Vlag gezet door LPTIM1 ISR om de main loop te signaleren.
 *          volatile: gewijzigd in ISR-context, geëvalueerd in main-context. */
static volatile uint8_t g_lptim_wakeup_flag = 0U;

/** @brief  Opgeslagen basislijn ADC-waarde van de SPEC alcoholsensor in schone
 *          lucht (virtuele aarde offset ~94 mV = ~7-8 LSB). Eenmalig bepaald
 *          tijdens opstartkalibratie (auto-zeroing). */
static uint8_t baseline_offset = 0U;

/* ═══════════════════════════════════════════════════════════════════════════
 * PERIFERIE-HANDLES (file scope)
 * ═══════════════════════════════════════════════════════════════════════════ */
static ADC_HandleTypeDef   hadc;
static LPTIM_HandleTypeDef hlptim1;
static UART_HandleTypeDef  huart2;

/* ═══════════════════════════════════════════════════════════════════════════
 * FORWARD DECLARATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Systeem & Klok */
static void SystemClock_Config(void);
static void SystemClock_Restore_AfterStop(void);

/* GPIO */
static void GPIO_Init_AllPins_Analog(void);
static void PA2_Mode_Analog(void);
static void PA2_Mode_UART_TX(void);

/* Periferie-initialisatie */
static void MX_ADC_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_USART2_Init(void);

/* Applicatielogica */
static uint8_t  ADC_Read_SingleChannel(uint32_t channel);
static void     Calibrate_Baseline(void);
       uint16_t bereken_ethanol_ppm(uint8_t adc_alc, uint8_t adc_ntc);
static void     Transmit_Result(uint16_t ppm);


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                        INTERRUPT HANDLERS                              ║
 * ║  Geïntegreerd in main.c voor een zelfstandig, gebruiksklaar bestand    ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  SysTick interrupt handler (1 ms tick voor HAL timebase).
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief  LPTIM1 interrupt handler.
 *         Routeert naar HAL die HAL_LPTIM_AutoReloadMatchCallback aanroept.
 */
void LPTIM1_IRQHandler(void)
{
    HAL_LPTIM_IRQHandler(&hlptim1);
}

/**
 * @brief  LPTIM1 AutoReload Match callback.
 *         Aangeroepen door HAL wanneer de teller de ARR-waarde bereikt.
 *
 * ONTWERPKEUZE: De callback zet ALLEEN een vlag; GEEN verwerking in ISR-
 * context. Dit is het "minimal-ISR-latency" patroon: alle berekeningen
 * worden uitgesteld naar de main-context om de ISR zo kort mogelijk te
 * houden (< 1 µs → minimaal dynamisch vermogen in interrupt-context).
 * Ref: Dictaat §1.10.1 & §3.3.1
 */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim->Instance == LPTIM1)
    {
        g_lptim_wakeup_flag = 1U;
    }
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                       KLOKCONFIGURATIE                                 ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Configureer systeemklok: MSI op 2.097 MHz (Range 5), LSI voor LPTIM1.
 *
 * ONTWERPKEUZE – Waarom MSI 2.1 MHz i.p.v. HSI 16 MHz?
 * Dynamisch vermogen schaalt lineair met frequentie:
 *     P_dyn = C × V² × f
 * Bij 2.1 MHz vs 16 MHz reduceren we P_dyn met factor ~7.6×.
 *
 * Waarom geen PLL?
 * De PLL zelf verbruikt ~100 µA statisch; niet nodig bij MSI-directe klok.
 *
 * Voltage Scaling Range 3 (1.2V kern):
 * Minimaliseert V² bijdrage. Toegestaan tot 4.2 MHz bij 0 Flash wait states.
 *
 * Ref: Dictaat §3.3.1 "Gedissipeerd vermogen tijdens schakelen"
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef  osc_init = {0};
    RCC_ClkInitTypeDef  clk_init = {0};

    /* PWR-klok inschakelen (nodig voor voltage scaling) */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Kernspanning naar Scale 3 (1.2V) – laagste dynamisch vermogen.
     * Maximaal 4.2 MHz met 0 Flash wait states. */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /* Wacht tot spanningsregelaar stabiel is */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) { /* hardware sync */ }

    /* ── Oscillator Configuratie ──────────────────────────────────────── */
    osc_init.OscillatorType = RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_LSI;

    /* MSI: 2.097 MHz (Range 5) – hoofdsysteemklok */
    osc_init.MSIState            = RCC_MSI_ON;
    osc_init.MSIClockRange       = RCC_MSIRANGE_5;
    osc_init.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;

    /* LSI: ~37 kHz – klokbron voor LPTIM1 in Stop Mode */
    osc_init.LSIState = RCC_LSI_ON;

    /* PLL: UIT – niet nodig, bespaart ~100 µA */
    osc_init.PLL.PLLState = RCC_PLL_NONE;

    HAL_RCC_OscConfig(&osc_init);

    /* ── Busklok Configuratie ─────────────────────────────────────────── */
    clk_init.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;      /* HCLK  = 2.097 MHz */
    clk_init.APB1CLKDivider = RCC_HCLK_DIV1;         /* PCLK1 = 2.097 MHz */
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;         /* PCLK2 = 2.097 MHz */

    HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0); /* 0 WS bij ≤4.2 MHz */

    /* Schakel HSI uit als HAL_Init() deze heeft ingeschakeld (HSI ≈ 45 µA) */
    __HAL_RCC_HSI_DISABLE();
}

/**
 * @brief  Herstel systeemklok na ontwaken uit Stop Mode.
 *
 * Na Stop Mode exit is MSI automatisch geselecteerd als systeemklok op de
 * range die vóór Stop was ingesteld (RCC_ICSCR wordt bewaard in Stop Mode).
 * Er is geen PLL- of HSI-herconfiguratie nodig.
 */
static void SystemClock_Restore_AfterStop(void)
{
    RCC_ClkInitTypeDef clk_init = {0};
    clk_init.ClockType    = RCC_CLOCKTYPE_SYSCLK;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0);
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                        GPIO CONFIGURATIE                               ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Initialiseer ALLE GPIO-pinnen als Analog (geen pull) om
 *         lekstroom te minimaliseren.
 *
 * ONTWERPKEUZE: Een zwevende digitale ingang schakelt ongedefinieerd tussen
 * hoog en laag door thermische ruis en elektromagnetische koppeling, wat
 * shoot-through stromen veroorzaakt in de ingangs-Schmitt-trigger (~1 µA/pin).
 * Analog Mode schakelt de digitale ingangsbuffer fysiek uit.
 * Ref: Dictaat §3.3.3 "Statisch vermogen & GPIO lekstromen"
 *
 * @note   PA13/PA14 (SWD) worden ook Analog gezet in PRODUCTION_MODE.
 *         Tijdens ontwikkeling overschrijft de debugger deze pinnen.
 */
static void GPIO_Init_AllPins_Analog(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* GPIO-klokken inschakelen voor configuratie */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /* ── Port A ─────────────────────────────────────────────────────── */
#if PRODUCTION_MODE
    gpio.Pin = GPIO_PIN_All;           /* Inclusief PA13/PA14 (SWD)      */
#else
    gpio.Pin = GPIO_PIN_All & ~(GPIO_PIN_13 | GPIO_PIN_14); /* SWD bewaren */
#endif
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ── Port B ─────────────────────────────────────────────────────── */
    gpio.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ── Port C (indien beschikbaar op package) ────────────────────── */
    gpio.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Schakel ongebruikte GPIO-klokken uit om dynamisch vermogen te besparen.
     * Houd alleen GPIOA ingeschakeld (PA1 en PA2). */
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
}

/**
 * @brief  Configureer PA2 als Analog (voor ADC_IN2 of Stop Mode).
 *         Schakelt de digitale output-driver uit → geen lekstroom.
 */
static void PA2_Mode_Analog(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief  Configureer PA2 als USART2_TX (AF4, push-pull) voor aansturing van
 *         de analoge RF-zender (MAX2661 mixer + BSS138 ASK/OOK poort).
 *
 * ONTWERPKEUZE: GPIO-snelheid op LOW om edge rate en daarmee
 * schakelstroom (P_schakel = α·C·V²·f) en EMI te minimaliseren.
 * Bij 2400 baud (~416.7 µs/bit) is low-speed GPIO ruim voldoende.
 */
static void PA2_Mode_UART_TX(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_2;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                        ADC CONFIGURATIE                                ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Initialiseer ADC1 in ultra-low-power 8-bit Auto-Off mode.
 *
 * ONTWERPVERANTWOORDING (Ref: Dictaat §3.4 "Low-power beperkingen ADC"):
 *
 * 1) 8-bit resolutie (i.p.v. 12-bit):
 *    - Conversietijd: 8.5 ADC-cycli vs 12.5 → 32% sneller.
 *    - Kwantiseringsstap Q = 3300 mV / 255 = 12.94 mV.
 *    - Voor een sensor met herhaalbaarheid ±3% en analoge TIA-ruis is
 *      8-bit (0.39% stapgrootte) ruim toereikend zonder onnodig energie
 *      te verspillen aan het kwantiseren van ruis (§3.4.1).
 *    - Kortere conversie = kortere actieve tijd = lager energieverbruik.
 *
 * 2) Auto-Off Mode (AUTOFF=1):
 *    - ADC analoge circuitry (~50 µA) schakelt ALLEEN in tijdens conversie.
 *    - Automatische uitschakeling na elke conversie.
 *    - Elimineert ADC statische stroom tussen metingen.
 *
 * 3) Auto-Wait Mode (AUTDLY=1):
 *    - Voorkomt data-overrun zonder actieve CPU polling loops.
 *    - Volgende conversie start pas wanneer vorige data gelezen is.
 *
 * 4) Sampling Time = 160.5 cycli:
 *    - Vereist voor hoogohmige bron (1 MΩ NTC thermistor).
 *    - T_sample = 160.5 / 2.097 MHz ≈ 76.5 µs.
 */
static void MX_ADC_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc.Instance = ADC1;

    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;   /* 2.097 MHz   */
    hadc.Init.Resolution            = ADC_RESOLUTION_8B;            /* 8-bit: 0–255*/
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode          = ADC_SCAN_DISABLE;             /* Enkel kanaal*/
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait      = ENABLE;                       /* AUTDLY bit  */
    hadc.Init.LowPowerAutoPowerOff  = ENABLE;                       /* AUTOFF bit  */
    hadc.Init.ContinuousConvMode    = DISABLE;                      /* Single-shot */
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;   /* Max sample  */

    HAL_ADC_Init(&hadc);

    /* Interne zelfkalibratie: elimineert ADC-offsetfouten.
     * Draait slechts éénmaal bij koude opstart. */
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                       LPTIM1 CONFIGURATIE                              ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Initialiseer LPTIM1 als 1 Hz wektimer vanuit Stop Mode.
 *
 * ONTWERPKEUZE: LPTIM1 wordt geklokt door LSI (~37 kHz) die actief blijft
 * in Stop Mode. Hierdoor is LPTIM1 de ENIGE eventbron in het systeem.
 * De CPU is volledig stilgelegd (WFI) tot LPTIM1 een ARRM-interrupt
 * genereert, wat een zuiver event-driven architectuur afdwingt.
 * Ref: Dictaat §1.2.1 & §1.10.1 "Event-driven state machine"
 */
static void MX_LPTIM1_Init(void)
{
    /* ── RCC: Selecteer LSI als LPTIM1 klokbron ──────────────────────── */
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    pclk.LptimClockSelection  = RCC_LPTIM1CLKSOURCE_LSI;
    HAL_RCCEx_PeriphCLKConfig(&pclk);

    __HAL_RCC_LPTIM1_CLK_ENABLE();

    /* ── LPTIM1 initialisatie ────────────────────────────────────────── */
    hlptim1.Instance = LPTIM1;

    hlptim1.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;   /* Geen prescaler  */
    hlptim1.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim1.Init.OutputPolarity  = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptim1.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
    hlptim1.Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;

    HAL_LPTIM_Init(&hlptim1);

    /* ── NVIC: Schakel LPTIM1 interrupt in (wekt CPU uit Stop) ───────── */
    HAL_NVIC_SetPriority(LPTIM1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                       USART2 CONFIGURATIE                              ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Initialiseer USART2 voor aansturing van de discrete analoge RF-zender
 *         (MAX2661 mixer + BSS138 ASK/OOK) op 2400 baud.
 *
 * ONTWERPVERANTWOORDING VOLGENS DICTAAT (§1.4.1, §1.4.2 & §1.5):
 * 1. Waarom 2400 baud i.p.v. hoge baudrate?
 *    De analoge RF-ontvanger gebruikt een discrete omhullende detector / diode-
 *    detector (§1.5.1). De gevoeligheid van zo'n ontvanger wordt volgens (1.6)
 *    bepaald door de ruisbandbreedte B:
 *        S_or = kT · B · F_T · SNR_min
 *    Bij 2400 baud is de benodigde filterbandbreedte B ruwweg 48× kleiner dan
 *    bij 115200 baud. Dit verlaagt de thermische ruisvloer met 16.8 dB!
 *    Hierdoor kan met het bescheiden zendvermogen van de discrete MAX2661 mixer
 *    toch de vereiste SNR en bitfoutkans (Pe, tabel 1.3) worden gehaald.
 *
 * 2. USART2 baudrate-generatie:
 *    Met MSI op 2.097 MHz: BRR = 2097152 / 2400 ≈ 873.81.
 *    Dit geeft een frequentiefout < 0.05%, wat ruim binnen de tolerantie valt.
 *
 * 3. TX-only mode:
 *    De ontvanger-circuits van de USART zijn hardwarematig uitgeschakeld,
 *    wat ~10 µA bespaart in actieve toestand.
 */
static void MX_USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;

    huart2.Init.BaudRate     = 2400;                     /* 2400 baud voor analoge RF-link */
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX;             /* Alleen TX       */
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart2);
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                      ADC KANAAL UITLEZING                              ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Lees een enkel ADC-kanaal in 8-bit Auto-Off mode.
 *
 * @param  channel  ADC-kanaal (ADC_CHANNEL_1 of ADC_CHANNEL_2)
 * @retval uint8_t  8-bit ADC-waarde (0–255)
 *
 * @note   Met AUTOFF=1 schakelt de ADC automatisch in voor deze conversie
 *         en weer uit daarna (nul statische stroom tussendoor).
 *
 * @note   PollForConversion is een hardware-gesynchroniseerde wacht (~80 µs
 *         voor 8-bit bij 2.1 MHz). Dit is de minimale fysieke SAR-conversietijd.
 */
static uint8_t ADC_Read_SingleChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};

    /* Wis ALLE kanaalselecties om single-channel conversie te garanderen.
     * Op STM32L0 is CHSELR een bitmask: meerdere gezette bits → scan mode.
     * We wissen dit vóór het selecteren van het nieuwe kanaal.
     * CHSELR mag alleen gewijzigd worden als ADEN=0, wat gegarandeerd is
     * door Auto-Off (automatische uitschakeling na vorige conversie). */
    hadc.Instance->CHSELR = 0U;

    /* Selecteer het gewenste kanaal */
    cfg.Channel = channel;
    cfg.Rank    = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &cfg);

    /* Start conversie (AUTOFF: ADC schakelt automatisch in) */
    HAL_ADC_Start(&hadc);

    /* Wacht op conversie-einde (~80 µs bij 2.1 MHz, 8-bit + 160.5 sample) */
    HAL_ADC_PollForConversion(&hadc, 2U);

    /* Lees resultaat (alleen onderste 8 bits geldig bij 8-bit resolutie) */
    uint8_t value = (uint8_t)(HAL_ADC_GetValue(&hadc) & 0xFFU);

    /* Stop ADC (AUTOFF heeft al uitgeschakeld, ruim HAL-state op) */
    HAL_ADC_Stop(&hadc);

    return value;
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                    AUTO-ZEROING KALIBRATIE                             ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Bepaal de basislijn (DC virtuele aarde offset ~94 mV) van de
 *         SPEC alcoholsensor in schone lucht.
 *
 * ONTWERPKEUZE: De auto-zeroing techniek samplet de sensoruitgang onder
 * bekende schone-lucht condities en slaat het gemiddelde op in baseline_offset.
 * De elektrochemische SPEC-sensor heeft een virtuele aardebias van ~94 mV
 * (overeenkomend met ~7-8 LSB bij 1 LSB = 12.94 mV). Alle volgende metingen
 * trekken deze offset af, waardoor een zuiver nul-gerefereerd signaal ΔV
 * ontstaat dat recht evenredig is met de ethanolconcentratie.
 *
 * Deling door 8 wordt geïmplementeerd als right-shift (>>3), wat compileert
 * naar 1 CPU-cyclus vs ~20 cycli voor een software-deling op Cortex-M0+.
 * Ref: Dictaat §1.10.2 & §3.3 "Kortstlopende code minimaliseert energie"
 *
 * @pre    Moet worden aangeroepen met sensor in schone lucht (geen ethanol).
 * @pre    ADC moet geïnitialiseerd zijn (MX_ADC_Init aangeroepen).
 */
static void Calibrate_Baseline(void)
{
    uint16_t accumulator = 0U;

    for (uint8_t i = 0U; i < CALIBRATION_SAMPLES; i++)
    {
        accumulator += ADC_Read_SingleChannel(ADC_CH_ALCOHOL);
    }

    /* Gehele deling door macht-van-2 via bit-shift:
     * accumulator / 8 == accumulator >> 3 */
    baseline_offset = (uint8_t)(accumulator >> CALIBRATION_SHIFT);
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║              ETHANOL PPM BEREKENING (UITSLUITEND INTEGERS)             ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Bereken temperatuur-gecompenseerde ethanolconcentratie in PPM.
 *
 * ONTWERPVERANTWOORDING VOLGENS DICTAAT (Dr. Ing. M.J. van der Horst):
 *
 * 1. TIA DYNAMIC RANGE MATCHING (§3.1.2 & §3.1.5):
 *    De SPEC 3SP-Ethanol-1000 sensor produceert een stroom I_sensor evenredig
 *    met de ethanolconcentratie (nominaal 15 nA/ppm, max 21 nA/ppm door spreiding).
 *    De transimpedantieversterker zet deze stroom om via terugkoppelweerstand Rf = 150 kΩ:
 *        ΔV = I_sensor × Rf
 *    Bij de maximale meetconcentratie van 1000 ppm en worst-case gevoeligheid (21 nA/ppm):
 *        I_sensor,max = 1000 ppm × 21 nA/ppm = 21 µA
 *        V_out,max = V_bias + (21 µA × 150 kΩ) = 94 mV + 3150 mV = 3244 mV (3.244 V)
 *    Dit benut het 3.3V ADC-bereik (3300 mV) optimaal (98.3% uitsturing) zonder dat
 *    de opamp clipt of de 8-bit ADC in verzadiging (code 255) raakt.
 *
 * 2. KORTSTOPENDE INTEGER WISKUNDE (§1.10.2 & §3.5):
 *    De Cortex-M0+ bezit geen FPU hardware. Drijvende-komma berekeningen genereren
 *    honderden extra instructies en verlengen de actieve runtime (t_actief).
 *    Door algebraïsche vereenvoudiging van de fysieke formule ontstaat één compacte
 *    32-bit integer breuk zonder floating-point bibliotheek overhead:
 *
 *    Fysische afleiding:
 *      (a) Spanning:    ΔV [mV] = adc_corrected × (3300 / 255)
 *      (b) Stroom:      I_sensor [µA] = ΔV / 150  (mV / kΩ = µA)
 *      (c) Stroom nA:   I_sensor [nA] = I_sensor [µA] × 1000 = ΔV × (1000 / 150) = ΔV × (20 / 3)
 *      (d) Concentratie: PPM = I_sensor [nA] / SENSITIVITY
 *
 *    Combinatie tot één breuk:
 *      PPM_raw = (adc_corrected × 3300 × 20) / (255 × 3 × SENSITIVITY)
 *              = (adc_corrected × 66000) / (765 × SENSITIVITY)
 *    Wegdelen van gemene deler 15 (66000/15 = 4400, 765/15 = 51):
 *      PPM_raw = (adc_corrected × 4400) / (51 × SENSITIVITY)
 *
 * 3. OVERFLOW & PRECISIE GARANTIE:
 *    - Max tellerwaarde: 255 × 4400 = 1.122.000 (past ruim in uint32_t, max 4.29×10^9).
 *    - Deling als allerlaatste bewerking behoudt maximale resolutie zonder afrondverlies.
 *
 * @param  adc_alc  Ruwe 8-bit ADC-waarde van alcoholsensor TIA (Kanaal 1)
 * @param  adc_ntc  Ruwe 8-bit ADC-waarde van NTC thermistor (Kanaal 2)
 * @retval uint16_t Temperatuur-gecompenseerde ethanolconcentratie in PPM
 */
uint16_t bereken_ethanol_ppm(uint8_t adc_alc, uint8_t adc_ntc)
{
    /* ── Stap 1: Basislijn-aftrek met Underflow-protectie ────────────── */
    uint8_t adc_corrected;

    if (adc_alc > baseline_offset)
    {
        adc_corrected = adc_alc - baseline_offset;
    }
    else
    {
        /* Sensoruitgang ≤ basislijn: geen ethanol gedetecteerd.
         * Retourneer direct 0 om onnodige CPU-cycli te vermijden
         * ("kortstlopende code" = minimale energie per meting). */
        return 0U;
    }

    /* ── Stap 2: Berekening PPM_raw via Afgeleide TIA Formule ─────────── */
    /* Gecombineerde 32-bit breuk: (adc_corrected × 4400) / (51 × SENSITIVITY) */
    uint32_t ppm_raw = ((uint32_t)adc_corrected * 4400U) / (51U * SENSOR_SENSITIVITY_NA_PPM);

    /* ── Stap 3: Temperatuur opzoeken via NTC LUT ────────────────────── */
    /* De NTC LUT-waarden zijn monotoon dalend:
     *   ntc_lut[0]  = 206  (0°C,  hoge R → hoge ADC-waarde)
     *   ntc_lut[40] =  82  (40°C, lage R  → lage ADC-waarde)
     *
     * Zoek de eerste index waar adc_ntc >= ntc_lut[i]. */
    uint8_t temp_c = 25U;   /* Default: 25°C referentietemperatuur */

    for (uint8_t i = 0U; i < NTC_LUT_SIZE; i++)
    {
        if (adc_ntc >= ntc_lut[i])
        {
            temp_c = i;
            break;
        }
    }

    /* Begrenzing wanneer temperatuur boven 40°C ligt (lage NTC ADC-waarde) */
    if (adc_ntc < ntc_lut[NTC_LUT_SIZE - 1U])
    {
        temp_c = (uint8_t)(NTC_LUT_SIZE - 1U);   /* Begrens op 40°C */
    }

    /* ── Stap 4: Fixed-Point Temperatuurcompensatie (Q8.8) ───────────── */
    /* Elektrochemische sensoren vertonen temperatuurafhankelijke gevoeligheid.
     * Compensatiemodel in Q8.8 fixed-point (waarbij 256 ≡ 1.00×):
     *   factor = 256 + (25 - temp) × 5
     *
     * Bij T = 25°C: factor = 256 (1.00×)
     * Bij T < 25°C: gevoeligheid daalt → factor stijgt → compensatie verhoogt PPM
     * Begrensd op [128, 512] → [0.5×, 2.0×] ter bescherming tegen uitschieters. */
    int16_t comp_factor = (int16_t)256
                        + ((int16_t)25 - (int16_t)temp_c) * (int16_t)5;

    if (comp_factor < 128)  { comp_factor = 128; }
    if (comp_factor > 512)  { comp_factor = 512; }

    /* ── Stap 5: Toepassen Temperatuurcompensatiefactor ───────────────── */
    /* Q8.8 normalisatie: (ppm_raw × 256) / comp_factor.
     * Shift naar links (<< 8) is equivalent met vermenigvuldigen met 256.
     * Max tussenwaarde: ~1466 × 256 = 375.296 (past probleemloos in uint32_t). */
    uint16_t ppm = (uint16_t)(((uint32_t)ppm_raw << 8) / (uint32_t)comp_factor);

    return ppm;
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║          RF RESULTAAT TRANSMISSIE (OOK/ASK MET MAX2661 / BSS138)       ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Formatteer en verstuur RF datapakket via USART2 TX (PA2) naar de
 *         discrete analoge On-Off Keying (ASK/OOK) zender.
 *
 * ONTWERPVERANTWOORDING & TRADE-OFF VOLGENS DICTAAT (Dr. Ing. M.J. van der Horst):
 *
 * 1. TRADE-OFF: LAGE BAUDRATE VS. RF-LINK BETROUWBAARHEID (§1.4 t/m §1.6):
 *    Normaal dicteert de ultra-low-power vuistregel "race-to-sleep" (vgl. 1.10:
 *    E_z ≈ U · I_z · l / DR) een zo hoog mogelijke baudrate om actieve tijd te
 *    minimaliseren.
 *    Echter: bij een discrete analoge zender (MAX2661 mixer geschakeld door BSS138)
 *    en een analoge omhullende detectorontvanger (§1.5.1) is de gevoeligheid
 *    S_or volgens formule (1.6) omgekeerd evenredig met de ruisbandbreedte:
 *        S_or = kT · B · F_T · SNR_min
 *    Een lage datasnelheid (2400 baud) verkleint de benodigde detectorbandbreedte B
 *    met een factor ~48 ten opzichte van 115200 baud. Dit reduceert de thermische
 *    ruisvloer met 16.8 dB! De resulterende stijging in linkmarge is noodzakelijk
 *    om de gewenste bitfoutkans (Pe, tabel 1.3) te garanderen over een redelijke afstand.
 *
 * 2. PREAMBLE (4× 0xAA) EN SYNC BYTE (0xFF):
 *    Een analoge OOK-ontvanger gebruikt een diode- of MOSFET-detector in het
 *    subthresholdgebied met een piekgelijkrichter / RC-filter (§1.5.1, fig 1.8).
 *    Bij langere stilte zakt de slicing-drempel (decision threshold) in door ontlading.
 *    - 4× 0xAA (binair 10101010) wekt de analoge LNA/AGC en laadt de slicing condensator
 *      op naar de exacte 50% beslissingsdrempel (16.7 ms periodiek blokgolfsignaal).
 *    - 1× 0xFF (Start Frame Delimiter) signaleert eenduidig het einde van de
 *      synchronisatie en het begin van de geldige payload.
 *
 * 3. KORTE PAYLOAD ("%u\n"):
 *    Bij 2400 baud duurt 1 byte ca. 4.167 ms (1 start + 8 data + 1 stop = 10 bits).
 *    Om de "airtime" en daarmee het actieve energieverbruik van zowel de MCU als
 *    de MAX2661 RF-eindtrap te minimaliseren, wordt alle overbodige tekst
 *    geëlimineerd. Een meetwaarde van bijv. 250 ppm resulteert in `"250\n"` (4 bytes).
 *    Totale framegrootte = 4 (preamble) + 1 (sync) + 4 (payload) = 9 bytes (~37.5 ms).
 *
 * 4. HAL_UART_TRANSMIT TIMEOUT (100 ms):
 *    Bij 2400 baud duurt 10 bytes ~42 ms. De standaard 5 ms timeout zou de
 *    transmissie voortijdig afkappen vóór de eerste data-bits de TX-lijn verlaten!
 *    Een timeout van minimaal 100U biedt veilige marge en voorkomt communicatie-uitval.
 *
 * @param  ppm  Berekende temperatuur-gecompenseerde ethanolconcentratie in PPM
 */
static void Transmit_Result(uint16_t ppm)
{
    /* Packet buffer:
     *   [0..3] : 4 bytes Preamble (0xAA)
     *   [4]    : 1 byte Sync / Start-delimiter (0xFF)
     *   [5..n] : Korte geformatteerde ASCII payload ("%u\n")
     * Max lengte uint16_t (65535) + '\n' + '\0' = 7 bytes.
     * Totale pakketbuffer van 16 bytes is ruim toereikend. */
    uint8_t tx_packet[16];

    /* ── Stap 1: Vul Preamble in (4× 0xAA = 0b10101010) ─────────────── */
    tx_packet[0] = RF_PREAMBLE_BYTE;
    tx_packet[1] = RF_PREAMBLE_BYTE;
    tx_packet[2] = RF_PREAMBLE_BYTE;
    tx_packet[3] = RF_PREAMBLE_BYTE;

    /* ── Stap 2: Vul Start / Sync Byte in (0xFF) ─────────────────────── */
    tx_packet[4] = RF_SYNC_BYTE;

    /* ── Stap 3: Formatteer Korte Payload (alleen PPM + newline) ─────── */
    int payload_len = snprintf((char *)&tx_packet[5], sizeof(tx_packet) - 5U,
                               "%u\n", (unsigned int)ppm);

    if (payload_len <= 0)
    {
        return; /* Fout bij formattering → annuleer transmissie */
    }

    uint16_t total_len = (uint16_t)(5U + (uint16_t)payload_len);

    /* ── Stap 4: Time-multiplexing PA2 → Schakel naar UART TX (AF4) ──── */
    PA2_Mode_UART_TX();

    /* ── Stap 5: Verzend compleet RF frame over PA2 naar MAX2661/BSS138 ─ */
    /* Bij 2400 baud duurt een 10-byte frame ca. 41.7 ms.
     * Timeout ingesteld op 100U (100 ms) om afkapping te voorkomen. */
    HAL_UART_Transmit(&huart2, tx_packet, total_len, 100U);

    /* ── Stap 6: Zet PA2 direct terug naar Analog Mode ───────────────── */
    /* Elimineert CMOS-stuurstroom en lekstroom in Stop Mode en bereidt
     * PA2 voor op ADC_IN2 (NTC) uitlezing in de volgende meetcyclus. */
    PA2_Mode_Analog();
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                              MAIN                                      ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Applicatie-ingangspunt.
 *
 * Initialisatie → Kalibratie → Oneindige Stop/Wake cyclus.
 *
 * ARCHITECTUUR (Event-Driven State Machine):
 *
 *   INIT → CALIBREER → ┌─→ STOP MODE (WFI, 0.35 µA)
 *                       │         │ LPTIM1 IRQ (1 Hz)
 *                       │         ▼
 *                       │   LEES ADC (Kanaal 1: SPEC TIA, Kanaal 2: NTC)
 *                       │         │
 *                       │         ▼
 *                       │   BEREKEN PPM (integer breuk, LUT, Q8.8 compensatie)
 *                       │         │
 *                       │         ▼
 *                       │   RF TRANSMIT (PA2: analog → AF4 → 2400 baud ASK → analog)
 *                       │         │
 *                       └──── RACE TO SLEEP ──┘
 */
int main(void)
{
    /* ══════════════════════════════════════════════════════════════════
     * FASE 1: SYSTEEM INITIALISATIE
     * ══════════════════════════════════════════════════════════════════ */

    /* Initialiseer HAL (SysTick, Flash prefetch, NVIC groepen) */
    HAL_Init();

    /* Configureer systeemklok: MSI 2.097 MHz + LSI voor LPTIM1 */
    SystemClock_Config();

    /* Zet ALLE GPIO-pinnen op Analog mode ALS EERSTE om lekstroom
     * door zwevende ingangen te voorkomen vanaf power-on. */
    GPIO_Init_AllPins_Analog();

    /* Initialiseer periferie */
    MX_ADC_Init();
    MX_LPTIM1_Init();
    MX_USART2_Init();    /* USART2 op 2400 baud; PA2 blijft in Analog mode */

    /* ══════════════════════════════════════════════════════════════════
     * FASE 2: OPSTARTKALIBRATIE (Auto-Zeroing)
     * ══════════════════════════════════════════════════════════════════
     * Bepaalt eenmalig de statische virtuele aarde offset (~94 mV)
     * in schone lucht (geen ethanol aanwezig). */
    Calibrate_Baseline();

    /* ══════════════════════════════════════════════════════════════════
     * FASE 3: START LPTIM1 (1 Hz periodieke wake-up)
     * ══════════════════════════════════════════════════════════════════ */
    HAL_LPTIM_Counter_Start_IT(&hlptim1, LPTIM_ARR_1HZ);

    /* ══════════════════════════════════════════════════════════════════
     * FASE 4: HOOFDLUS – Event-Driven Ultra-Low-Power Cyclus
     * ══════════════════════════════════════════════════════════════════
     *
     * Duty-cycle analyse (per 1 seconde periode):
     *   Actieve tijd:  ~42 ms  (ADC + berekening + RF TX @ 2400 baud)
     *   Stop-tijd:     ~958 ms
     *   Duty cycle:    4.2%
     *
     * Gemiddelde stroom bij 3.3V:
     *   I_stop   = 0.35 µA × 0.958 ≈ 0.34 µA
     *   I_actief = 1.5 mA  × 0.042 ≈ 63.0 µA
     *   I_gem    ≈ 63.3 µA → P_gem = 209 µW (0.21 mW)  ≪  1 mW budget ✓
     *
     * Ref: Dictaat §1.2.1 "Powermanagement & Duty-cycle optimalisatie"
     * ══════════════════════════════════════════════════════════════════ */
    while (1)
    {
        /* Suspendeer SysTick vóór Stop om ongewenste wake-ups te voorkomen.
         * SysTick draait op 1 kHz; zonder suspenderen wekt deze de CPU
         * elke 1 ms, waardoor Stop Mode tenietgedaan wordt. */
        HAL_SuspendTick();

        /* ── Ga naar Stop Mode ──────────────────────────────────────────
         * Low-Power Regulator: reduceert regelaar-ruststroom naar ~0.35 µA.
         * WFI: CPU stopt tot interrupt (LPTIM1 ARRM match). */
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

        /* ── CPU wordt hier gewekt nadat LPTIM1 ISR is uitgevoerd ───────
         * De ISR heeft reeds g_lptim_wakeup_flag gezet.
         * MSI herstart automatisch op de geconfigureerde 2.097 MHz. */

        /* Herstel systeemklok (herselectie MSI als SYSCLK na wake-up) */
        SystemClock_Restore_AfterStop();

        /* Hervat SysTick (nodig voor UART timeout) */
        HAL_ResumeTick();

        /* ── Verwerk wake-up event ──────────────────────────────────────── */
        if (g_lptim_wakeup_flag != 0U)
        {
            g_lptim_wakeup_flag = 0U;

            /* 1) Lees ethanol TIA-uitgang (PA1 is permanent in Analog mode) */
            uint8_t adc_alcohol = ADC_Read_SingleChannel(ADC_CH_ALCOHOL);

            /* 2) Lees NTC thermistor (PA2 is in Analog mode → ADC_IN2) */
            uint8_t adc_ntc = ADC_Read_SingleChannel(ADC_CH_NTC);

            /* 3) Bereken temperatuur-gecompenseerde ethanol PPM via integer breuk */
            uint16_t ppm = bereken_ethanol_ppm(adc_alcohol, adc_ntc);

            /* 4) Verstuur resultaat via RF OOK/ASK (PA2: Preamble + Sync + "%u\n") */
            Transmit_Result(ppm);
        }

        /* Lus onmiddellijk terug naar Stop Mode (race-to-sleep).
         * Geen onnodige instructies tussen verwerking en slaapstand. */
    }
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                     HAL MSP INITIALISATIE                              ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  HAL MSP (MCU Support Package) initialisatie.
 *         Aangeroepen door HAL_Init() voor laagniveau klokconfiguratie.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}


/* ╔═════════════════════════════════════════════════════════════════════════╗
 * ║                         FOUTAFHANDELING                                ║
 * ╚═════════════════════════════════════════════════════════════════════════╝ */

/**
 * @brief  Minimale foutafhandelaar.
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* Halt. Debugger breekt hier. */
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  HAL assert handler (actief met USE_FULL_ASSERT).
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif /* USE_FULL_ASSERT */
