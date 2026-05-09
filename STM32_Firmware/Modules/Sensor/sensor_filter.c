#include "sensor_filter.h"

#include <stdlib.h>
#include <string.h>

#include "system_config.h"
#include "thresholds.h"

typedef struct
{
  float ring[SENSOR_FILTER_MEDIAN_SIZE];
  uint8_t ring_count;
  uint8_t ring_idx;
  float last_filtered;
  uint8_t have_filtered;
  uint8_t near_spike_count;
} sensor_filter_bank_t;

static sensor_filter_bank_t s_bank[SENSOR_FILTER_INSTANCE_COUNT];

static void sensor_filter_reset_one(sensor_filter_bank_t *b)
{
  b->ring_count = 0U;
  b->ring_idx = 0U;
  b->last_filtered = 0.0f;
  b->have_filtered = 0U;
  b->near_spike_count = 0U;
}

void sensor_filter_reset_all(void)
{
  uint8_t i;

  for (i = 0U; i < SENSOR_FILTER_INSTANCE_COUNT; i++)
  {
    sensor_filter_reset_one(&s_bank[i]);
  }
}

static int float_compare(const void *a, const void *b)
{
  float fa = *(const float *)a;
  float fb = *(const float *)b;

  if (fa < fb)
  {
    return -1;
  }

  if (fa > fb)
  {
    return 1;
  }

  return 0;
}

static float sensor_filter_median_of_bank(const sensor_filter_bank_t *b)
{
  float scratch[SENSOR_FILTER_MEDIAN_SIZE];

  if (b->ring_count == 0U)
  {
    return 0.0f;
  }

  (void)memcpy(scratch, b->ring, (size_t)b->ring_count * sizeof(float));
  qsort(scratch, b->ring_count, sizeof(float), float_compare);

  return scratch[b->ring_count / 2U];
}

static void sensor_filter_push_sample(sensor_filter_bank_t *b, float sample_cm)
{
  b->ring[b->ring_idx] = sample_cm;
  b->ring_idx = (uint8_t)((b->ring_idx + 1U) % SENSOR_FILTER_MEDIAN_SIZE);

  if (b->ring_count < SENSOR_FILTER_MEDIAN_SIZE)
  {
    b->ring_count++;
  }
}

void sensor_filter_commit(uint8_t sensor_index, distance_data_t *d)
{
  sensor_filter_bank_t *b;
  float raw_cm;
  float median_cm;
  float candidate_cm;
  float alpha;
  float ema_cm;

  if ((d == 0) || (sensor_index >= SENSOR_FILTER_INSTANCE_COUNT) ||
      (d->system_status != SYSTEM_STATUS_OK))
  {
    return;
  }

  b = &s_bank[sensor_index];
  raw_cm = d->distance_cm;

  if ((b->have_filtered != 0U) &&
      (b->last_filtered > (SAFE_DISTANCE_THRESHOLD_CM + 40.0f)) &&
      (raw_cm < WARNING_DISTANCE_THRESHOLD_CM))
  {
    b->near_spike_count++;

    if (b->near_spike_count < SENSOR_NEAR_SPIKE_CONFIRM_COUNT)
    {
      d->distance_cm = b->last_filtered;
      d->is_object_detected = 1U;
      d->system_status = SYSTEM_STATUS_OK;
      return;
    }
  }
  else
  {
    b->near_spike_count = 0U;
  }

  if (raw_cm < SENSOR_MIN_VALID_CM)
  {
    d->system_status = SYSTEM_STATUS_SENSOR_BELOW_MIN;
    d->distance_cm = SENSOR_MIN_VALID_CM;
    d->is_object_detected = 1U;
    return;
  }

  if (raw_cm > SENSOR_MAX_VALID_CM)
  {
    if (b->have_filtered != 0U)
    {
      d->distance_cm = b->last_filtered;
      d->is_object_detected = 1U;
      d->system_status = SYSTEM_STATUS_OK;
    }
    else
    {
      d->system_status = SYSTEM_STATUS_SENSOR_FAULT;
      d->distance_cm = 0.0f;
      d->is_object_detected = 0U;
    }

    return;
  }

  sensor_filter_push_sample(b, raw_cm);
  median_cm = sensor_filter_median_of_bank(b);

  candidate_cm = median_cm;

  if (b->have_filtered != 0U)
  {
    float step_cm = candidate_cm - b->last_filtered;

    if (step_cm > SENSOR_FILTER_MAX_STEP_CM)
    {
      candidate_cm = b->last_filtered + SENSOR_FILTER_MAX_STEP_CM;
    }
    else if (step_cm < (-SENSOR_FILTER_MAX_STEP_CM))
    {
      candidate_cm = b->last_filtered - SENSOR_FILTER_MAX_STEP_CM;
    }
  }

  alpha = SENSOR_FILTER_EMA_ALPHA;

  if (b->have_filtered == 0U)
  {
    ema_cm = candidate_cm;
  }
  else
  {
    ema_cm = (alpha * candidate_cm) + ((1.0f - alpha) * b->last_filtered);
  }

  b->last_filtered = ema_cm;
  b->have_filtered = 1U;
  d->distance_cm = ema_cm;
  d->is_object_detected = 1U;
  d->system_status = SYSTEM_STATUS_OK;
}
