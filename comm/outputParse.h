#ifndef OUTPUTPARSE_H
#define OUTPUTPARSE_H

uint16_t getTxSample(uint8_t *output, uint32_t *bitmask, uint16_t sampleNumber, uint16_t batch);
void loadBatch(void);
void flushSatQueue(void);
void memrcpy(void *dst, const void *src, size_t len);

#endif
