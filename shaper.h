// InputShaper.h
#pragma once
#ifndef shaper_H
#define shaper_H

#define SHAPER_NONE 0
#define SHAPER_ZV 1
#define SHAPER_ZVD 2
#define SHAPER_MZV 3
#define SHAPER_EI 4
// per 1000us
#define SAMPLING 800 

static volatile bool shaper_reset_requested;
class InputShaper {
private:
    static const uint8_t MAX_IMPULSES = 5;      // For ZV shaping
    static const uint16_t HISTORY_SIZE = 64;    // Buffer size for circular buffer
    
    struct Impulse {
        float amplitude;
        uint32_t timeDelay;
    };
    
    struct HistoryEntry {
        float velocity;
        uint32_t timeStamp;
    };
    
    // Circular buffer members
    Impulse impulses[MAX_IMPULSES];
    HistoryEntry history[HISTORY_SIZE];
    uint8_t head;     // Write position
    uint8_t tail;     // Read position
    uint8_t count;    // Number of valid entries
    uint8_t impulseCount;
    
    uint32_t microStepSize, currentTime;
    const float MICROS_TO_SECONDS = 1.0e-6f;

    uint32_t maxDelay;  // Cache the maximum delay
    
    uint32_t acV, sampleI, sampleC;

public:
    bool enabled;
    uint8_t shaperType;
    float naturalFrequency;
    float dampingRatio;    
    InputShaper(bool t) {
        enabled = t;
        shaperType = SHAPER_NONE;
        head = tail = count = 0;
    }

    void configure(float freq, float damp, uint8_t type) {
        naturalFrequency = freq;
        dampingRatio = damp;
        shaperType = type;
        enabled = (type != SHAPER_NONE);
        if (!enabled) return;

        float omega_d = sqrtf(1.0f - dampingRatio * dampingRatio);
        float K = expf(-dampingRatio * PI / omega_d);
        uint32_t td = 1000000 / (naturalFrequency * omega_d);
        
        // Recalculate shaper parameters
        switch(shaperType) {
            case SHAPER_ZV:  
                impulses[0] = {1.0f, 0};
                impulses[1] = {K, td};
                impulseCount = 2;
                maxDelay = td;
                break;
            case SHAPER_ZVD: 
                impulses[0] = {1.0f, 0};
                impulses[1] = {2.0f * K, td/2};
                impulses[2] = {K * K, td};
                impulseCount = 3;
                maxDelay = td;
                break;
            case SHAPER_MZV: 
                impulses[0] = {1-1/sqrtf(2), 0};
                impulses[1] = {(sqrtf(2)-1)*K, td*0.375f};
                impulses[2] = {(1-1/sqrtf(2))*K*K, td*0.75f};
                impulseCount = 3;
                maxDelay = td*0.75f;
                break;
            case SHAPER_EI:  
                impulses[0] = {0.25f*(1+0.05f), 0};
                impulses[1] = {0.5f*(1-0.05f)*K, td/2};
                impulses[2] = {0.25f*(1+0.05f)*K*K, td};
                impulseCount = 3;
                maxDelay = td;
                break;
            default: 
                impulseCount = 0;
                maxDelay = 0;
                break;
        }
        shaper_reset_requested = true;
    }

    void reset() {
        head = tail = count = 0;
        currentTime = 0;
        sampleI = sampleC = acV = 0;
        shaper_reset_requested = false;
    }
    
    
    float shapeVelocity(float velocityCommand) {
        if (!enabled || velocityCommand <= 0) {
            return velocityCommand;
        }

        if (shaper_reset_requested) reset();

        extern uint32_t stepdiv2;
        uint32_t stepTime = (velocityCommand > 0) ? (stepdiv2 / (0.078125f*velocityCommand)) : 0;
        
        // Accumulate samples
        sampleC += stepTime;
        sampleI++;
        acV += velocityCommand;

        // Add new entry to circular buffer
        if (sampleC > SAMPLING) {
            history[head] = {
                acV/float(sampleI),
                currentTime + sampleC/sampleI
            };
            
            head = (head + 1) % HISTORY_SIZE;
            if (count < HISTORY_SIZE) {
                count++;
            } else {
                tail = (tail + 1) % HISTORY_SIZE;
            }
            
            sampleC = 0;
            acV = 0;
            sampleI = 0;
        }

        // Remove old entries
        uint32_t cutoffTime = currentTime - maxDelay;
        while (count > 0 && history[tail].timeStamp < cutoffTime) {
            tail = (tail + 1) % HISTORY_SIZE;
            count--;
        }

        float shapedVelocity = 0.0f;
        float totalWeight = 0.0f;

        // Process each impulse
        for (uint8_t i = 0; i < impulseCount; i++) {
            uint32_t targetTime = currentTime - impulses[i].timeDelay;
            
            // Find entries in circular buffer
            int8_t beforeIndex = -1;
            int8_t afterIndex = -1;
            
            // Search through valid entries
            uint8_t searchCount = 0;
            uint8_t idx = tail;
            while (searchCount < count) {
                if (history[idx].timeStamp > targetTime) {
                    afterIndex = idx;
                    beforeIndex = (idx == tail) ? -1 : 
                                ((idx - 1 + HISTORY_SIZE) % HISTORY_SIZE);
                    break;
                }
                idx = (idx + 1) % HISTORY_SIZE;
                searchCount++;
            }

            if (beforeIndex >= 0 && afterIndex >= 0) {
                const HistoryEntry& beforeEntry = history[beforeIndex];
                const HistoryEntry& afterEntry = history[afterIndex];
                
                float timeFraction = float(targetTime - beforeEntry.timeStamp) / 
                                   float(afterEntry.timeStamp - beforeEntry.timeStamp);
                timeFraction = max(0.0f, min(1.0f, timeFraction));
                
                float interpolatedVel = beforeEntry.velocity + 
                    (afterEntry.velocity - beforeEntry.velocity) * timeFraction;
                
                shapedVelocity += impulses[i].amplitude * interpolatedVel;
                totalWeight += impulses[i].amplitude;
            } else if (count > 0) {
                // Use nearest available entry
                uint8_t nearestIdx = afterIndex >= 0 ? afterIndex : 
                                   ((head - 1 + HISTORY_SIZE) % HISTORY_SIZE);
                shapedVelocity += impulses[i].amplitude * history[nearestIdx].velocity;
                totalWeight += impulses[i].amplitude;
            }
        }
        
        currentTime += stepTime;
        return (totalWeight > 0) ? shapedVelocity / totalWeight : velocityCommand;
    }
};

extern InputShaper shaper;
#endif