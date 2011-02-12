extern uint16_t EEMEM EEbatchSampleStart;
extern uint16_t EEMEM EEbatchSampleEnd;
extern uint16_t EEMEM EEcurrentBatchNumber;
extern uint32_t EEMEM EEcurrentTelemetryBitmap[];
extern uint16_t EEMEM EEcommPromEnd;
extern uint16_t EEMEM EEcommPromStart;

uint16_t getTxSample(uint8_t *output, uint32_t *bitmask, uint16_t sampleNumber, uint16_t batch);
void loadBatch(void);
void flushSatQueue(void);
void memrcpy(void *dst, const void *src, size_t len);
