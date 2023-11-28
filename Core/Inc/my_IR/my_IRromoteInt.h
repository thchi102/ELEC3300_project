#ifndef _IR_REMOTE_INT_H
#define _IR_REMOTE_INT_H

#define MARK   1
#define SPACE  0

#define DISABLE_LED_FEEDBACK            false
#define ENABLE_LED_FEEDBACK             true
#define USE_DEFAULT_FEEDBACK_LED_PIN    0

#define RAW_BUFFER_LENGTH  100

#define IR_REC_STATE_IDLE      0 // Counting the gap time and waiting for the start bit to arrive
#define IR_REC_STATE_MARK      1 // A mark was received and we are counting the duration of it.
#define IR_REC_STATE_SPACE     2 // A space was received and we are counting the duration of it. If space is too long, we assume end of frame.
#define IR_REC_STATE_STOP      3 // Stopped until set to IR_REC_STATE_IDLE which can only be done by resume()

struct irparams_struct {
    // The fields are ordered to reduce memory over caused by struct-padding
    volatile uint8_t StateForISR;       ///< State Machine state
    uint_fast8_t IRReceivePin;          ///< Pin connected to IR data from detector
#if defined(__AVR__)
    volatile uint8_t *IRReceivePinPortInputRegister;
    uint8_t IRReceivePinMask;
#endif
    volatile uint_fast16_t TickCounterForISR; ///< Counts 50uS ticks. The value is copied into the rawbuf array on every transition.
#if !IR_REMOTE_DISABLE_RECEIVE_COMPLETE_CALLBACK
    void (*ReceiveCompleteCallbackFunction)(void); ///< The function to call if a protocol message has arrived, i.e. StateForISR changed to IR_REC_STATE_STOP
#endif
    bool OverflowFlag;                  ///< Raw buffer OverflowFlag occurred
#if RAW_BUFFER_LENGTH <= 254            // saves around 75 bytes program memory and speeds up ISR
    uint_fast8_t rawlen;                ///< counter of entries in rawbuf
#else
    uint_fast16_t rawlen;               ///< counter of entries in rawbuf
#endif
    uint16_t rawbuf[RAW_BUFFER_LENGTH]; ///< raw data / tick counts per mark/space, first entry is the length of the gap between previous and current command
};

#if (__INT_WIDTH__ < 32)
typedef uint32_t IRRawDataType;
#define BITS_IN_RAW_DATA_TYPE   32
#else
typedef uint64_t IRRawDataType;
#define BITS_IN_RAW_DATA_TYPE   64
#endif

/****************************************************
 *                     RECEIVING
 ****************************************************/

struct decode_results {
    decode_type_t decode_type;  // deprecated, moved to decodedIRData.protocol ///< UNKNOWN, NEC, SONY, RC5, ...
    uint16_t address;           // Used by Panasonic & Sharp [16-bits]
    uint32_t value;             // deprecated, moved to decodedIRData.decodedRawData ///< Decoded value / command [max 32-bits]
    uint8_t bits;               // deprecated, moved to decodedIRData.numberOfBits ///< Number of bits in decoded value
    uint16_t magnitude;         // deprecated, moved to decodedIRData.extra ///< Used by MagiQuest [16-bits]
    bool isRepeat;              // deprecated, moved to decodedIRData.flags ///< True if repeat of value is detected

// next 3 values are copies of irparams values - see IRremoteint.h
    uint16_t *rawbuf;       // deprecated, moved to decodedIRData.rawDataPtr->rawbuf ///< Raw intervals in 50uS ticks
    uint_fast8_t rawlen;        // deprecated, moved to decodedIRData.rawDataPtr->rawlen ///< Number of records in rawbuf
    bool overflow;              // deprecated, moved to decodedIRData.flags ///< true if IR raw code too long
};

struct IRrecv {
	
public:
    IRrecv();
    IRrecv(uint_fast8_t aReceivePin);
    IRrecv(uint_fast8_t aReceivePin, uint_fast8_t aFeedbackLEDPin);
    void setReceivePin(uint_fast8_t aReceivePinNumber);
    void registerReceiveCompleteCallback(void (*aReceiveCompleteCallbackFunction)(void));
    /*
     * Stream like API
     */
    void begin(uint_fast8_t aReceivePin, bool aEnableLEDFeedback = false, uint_fast8_t aFeedbackLEDPin =
    USE_DEFAULT_FEEDBACK_LED_PIN);
    void start();
    void enableIRIn(); // alias for start
    void start(uint32_t aMicrosecondsToAddToGapCounter);
    void startWithTicksToAdd(uint16_t aTicksToAddToGapCounter);
    void restartAfterSend();

    void addTicksToInternalTickCounter(uint16_t aTicksToAddToInternalTickCounter);
    void addMicrosToInternalTickCounter(uint16_t aMicrosecondsToAddToInternalTickCounter);

    bool available();
    IRData* read(); // returns decoded data
    // write is a method of class IRsend below
    // size_t write(IRData *aIRSendData, int_fast8_t aNumberOfRepeats = NO_REPEATS);
    void stop();
    void disableIRIn(); // alias for stop
    void end(); // alias for stop

    bool isIdle();

    /*
     * The main functions
     */
    bool decode();  // Check if available and try to decode
    void resume();  // Enable receiving of the next value

    /*
     * Useful info and print functions
     */
    void printIRResultMinimal(Print *aSerial);
    void printIRResultRawFormatted(Print *aSerial, bool aOutputMicrosecondsInsteadOfTicks = true);
    void printIRResultAsCVariables(Print *aSerial);
    uint32_t getTotalDurationOfRawData();

    /*
     * Next 4 functions are also available as non member functions
     */
    bool printIRResultShort(Print *aSerial, bool aPrintRepeatGap = true, bool aCheckForRecordGapsMicros = true);
    void printDistanceWidthTimingInfo(Print *aSerial, DistanceWidthTimingInfoStruct *aDistanceWidthTimingInfo);
    void printIRSendUsage(Print *aSerial);
#if defined(__AVR__)
    const __FlashStringHelper* getProtocolString();
#else
    const char* getProtocolString();
#endif
    static void printActiveIRProtocols(Print *aSerial);

    void compensateAndPrintIRResultAsCArray(Print *aSerial, bool aOutputMicrosecondsInsteadOfTicks = true);
    void compensateAndPrintIRResultAsPronto(Print *aSerial, uint16_t frequency = 38000U);

    /*
     * Store the data for further processing
     */
    void compensateAndStoreIRResultInArray(uint8_t *aArrayPtr);
    size_t compensateAndStorePronto(String *aString, uint16_t frequency = 38000U);

    /*
     * The main decoding functions used by the individual decoders
     */
    bool decodePulseDistanceWidthData(PulseDistanceWidthProtocolConstants *aProtocolConstants, uint_fast8_t aNumberOfBits,
            uint_fast8_t aStartOffset = 3);

    bool decodePulseDistanceWidthData(uint_fast8_t aNumberOfBits, uint_fast8_t aStartOffset, uint16_t aOneMarkMicros,
            uint16_t aZeroMarkMicros, uint16_t aOneSpaceMicros, uint16_t aZeroSpaceMicros, bool aMSBfirst);

    bool decodeBiPhaseData(uint_fast8_t aNumberOfBits, uint_fast8_t aStartOffset, uint_fast8_t aStartClockCount,
            uint_fast8_t aValueOfSpaceToMarkTransition, uint16_t aBiphaseTimeUnit);

    void initBiphaselevel(uint_fast8_t aRCDecodeRawbuffOffset, uint16_t aBiphaseTimeUnit);
    uint_fast8_t getBiphaselevel();

    /*
     * All standard (decode address + command) protocol decoders
     */
    bool decodeBangOlufsen();
    bool decodeBoseWave();
    bool decodeDenon();
    bool decodeFAST();
    bool decodeJVC();
    bool decodeKaseikyo();
    bool decodeLegoPowerFunctions();
    bool decodeLG();
    bool decodeMagiQuest(); // not completely standard
    bool decodeNEC();
    bool decodeRC5();
    bool decodeRC6();
    bool decodeSamsung();
    bool decodeSharp(); // redirected to decodeDenon()
    bool decodeSony();
    bool decodeWhynter();

    bool decodeDistanceWidth();

    bool decodeHash();

    // Template function :-)
    bool decodeShuzu();

    /*
     * Old functions
     */
    bool decodeDenonOld(decode_results *aResults);
    bool decodeJVCMSB(decode_results *aResults);
    bool decodeLGMSB(decode_results *aResults);
    bool decodeNECMSB(decode_results *aResults);
    bool decodePanasonicMSB(decode_results *aResults);
    bool decodeSonyMSB(decode_results *aResults);
    bool decodeSAMSUNG(decode_results *aResults);
    bool decodeHashOld(decode_results *aResults);

    bool decode_old(decode_results *aResults);

    bool decode(
            decode_results *aResults)
                    __attribute__ ((deprecated ("Please use IrReceiver.decode() without a parameter and IrReceiver.decodedIRData.<fieldname> .")));

    // for backward compatibility. Now in IRFeedbackLED.hpp
    void blink13(uint8_t aEnableLEDFeedback)
            __attribute__ ((deprecated ("Please use setLEDFeedback() or enableLEDFeedback() / disableLEDFeedback().")));

    /*
     * Internal functions
     */
    void initDecodedIRData();
    uint_fast8_t compare(uint16_t oldval, uint16_t newval);
    bool checkHeader(PulseDistanceWidthProtocolConstants *aProtocolConstants);
    void checkForRepeatSpaceTicksAndSetFlag(uint16_t aMaximumRepeatSpaceTicks);
    bool checkForRecordGapsMicros(Print *aSerial);

    IRData decodedIRData;       // New: decoded IR data for the application

    // Last decoded IR data for repeat detection and parity for Denon autorepeat
    decode_type_t lastDecodedProtocol;
    uint32_t lastDecodedAddress;
    uint32_t lastDecodedCommand;

    uint8_t repeatCount;        // Used e.g. for Denon decode for autorepeat decoding.
};
#endif