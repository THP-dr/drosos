/*
  Benchmark: Native C++ vs. ESP-DSP Library
  -----------------------------------------------
  This program compares the computational performance (in CPU cycles) between:
  1. Standard C++ code compiled with GCC (default settings).
  2. Hand-optimized Assembly routines from the Espressif ESP-DSP library.

  Goal: 
  Measure the time required to compute a single Dot Product of two 4,096-element 
  vectors (totaling 4,096 multiplications and 4,096 additions per test).

  Methodology:
  The test executes 10 iterations for each method. To account for interrupt 
  jitter or cache misses, only the 'best' (fastest) run for each method is 
  retained for the final comparison.
*/

//DSP stands for Digital Signal Processing, optimized for tasks where math is the bottleneck rather than memory speed
//these include dot products, fourier transforms, kalman filters and matrix multiplication
#include "esp_dsp.h" 

#define BUFFER_SIZE 4096  
#define NUM_RUNS 10

float vecA[BUFFER_SIZE] __attribute__((aligned(16)));
float vecB[BUFFER_SIZE] __attribute__((aligned(16)));

void setup() {
    Serial.begin(115200);
    delay(2000);

    for (int i = 0; i < BUFFER_SIZE; i++) {
        vecA[i] = (float)random(0, 100) / 10.0f;
        vecB[i] = (float)random(0, 100) / 10.0f;
    }

    Serial.println("--- Benchmark: Dot Product (Best of 10) ---");

    uint32_t best_native = 0xFFFFFFFF;
    uint32_t best_dsp = 0xFFFFFFFF;
    float result_native = 0;
    float result_dsp = 0;

    // --- TEST 1: Native C++ ---
    for (int r = 0; r < NUM_RUNS; r++) {
        float local_sum = 0;
        uint32_t start = ESP.getCycleCount();

        for (int i = 0; i < BUFFER_SIZE; i++) {
            local_sum += vecA[i] * vecB[i];
        }
        
        uint32_t diff = ESP.getCycleCount() - start;
        if (diff < best_native) best_native = diff;
        result_native = local_sum; 
    }

    // --- TEST 2: ESP-DSP ---
    for (int r = 0; r < NUM_RUNS; r++) {
        float local_sum = 0;
        uint32_t start = ESP.getCycleCount();

        dsps_dotprod_f32(vecA, vecB, &local_sum, BUFFER_SIZE);

        uint32_t diff = ESP.getCycleCount() - start;
        if (diff < best_dsp) best_dsp = diff;
        result_dsp = local_sum;
    }

    Serial.printf("Best Native Cycles: %u\n", best_native);
    Serial.printf("Best ESP-DSP Cycles: %u\n", best_dsp);
    
    Serial.printf("Verify: Native=%.2f, DSP=%.2f\n", result_native, result_dsp);

    if (best_dsp > 0) {
        float speedup = (float)best_native / best_dsp;
        Serial.printf("\n>>> ESP-DSP is %.2fx FASTER (Best Run)!\n", speedup);
    }
}

void loop() {}