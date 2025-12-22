#include <stdio.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// Callback function declaration
static int get_signal_data(size_t offset, size_t length, float *out_ptr);

// Raw features copied from test sample
static const float features[] = {
    // Copy raw features here (e.g. from the 'Model testing' page)
  -0.1400, -5.7800, 1.0400, -20.9300, -56.1000, -18.3200, -0.6900, -6.4400, 1.6000, -63.7900, -71.1600, -37.2200, -0.5900, -7.3300, 1.8100, -113.4900, -93.3000, -47.4600, -0.3500, -8.5800, 2.2700, -166.4100, -121.6500, -46.0600, -2.0400, -11.0100, 4.0900, -218.4900, -117.4400, -32.9500, -1.6000, -11.8200, 4.7100, -270.3800, -109.0300, -7.4700, -0.9900, -13.8400, 4.5100, -321.7800, -111.9500, 21.9200, -1.1000, -17.0800, 5.0800, -370.5600, -107.9300, 52.9000, -0.6400, -19.6500, 5.2800, -413.8500, -74.7000, 86.8000, -0.5800, -19.6500, 4.5700, -440.6200, -37.6300, 108.7500, -1.4400, -19.6500, 3.9400, -447.0800, 2.6800, 118.3300, 0.3300, -17.1800, 2.8300, -431.0400, 28.4100, 110.4600, 0.7900, -14.8100, 1.8300, -391.9600, 27.3100, 88.7500, 1.1100, -10.9000, 0.7200, -329.7700, 26.4600, 68.7500, 1.6600, -8.2100, -0.5200, -281.1700, 5.3000, 56.6800, 2.0500, -6.4400, -1.3800, -209.8900, -20.3100, 49.7300, 1.6900, -4.5300, -1.7400, -137.6900, -35.0600, 48.7500, 1.4100, -3.3900, -1.7200, -70.1900, -46.5900, 45.1000, 0.9300, -2.9700, -1.7500, -9.8900, -59.5200, 38.0800, 0.0100, -2.4900, -1.4600, 46.6300, -53.4800, 23.6900, -0.3500, -2.0400, -1.3600, 94.5000, -37.0800, 0.6400, -0.1400, -2.7400, -1.0800, 135.2300, -25.2500, -25.7600, -0.4700, -4.9200, -0.7200, 160.7200, 3.1100, -42.2200, 0.0700, -5.1000, -0.7800, 185.9000, 27.5000, -58.9300, 1.0800, -6.0900, -0.2700, 236.2700, 13.5300, -80.1500, -0.4000, -9.6900, 1.4400, 275.1700, 28.9000, -95.7000, -0.6400, -12.5200, 2.7500, 308.5200, 95.4200, -108.9300, 0.6900, -15.9600, 2.1300, 335.7200, 111.2100, -118.6800, 1.1700, -18.8900, 2.7200, 356.9400, 136.4600, -120.5100, 1.7600, -19.6500, 4.1700, 369.9200, 125.5400, -103.6200, -0.5400, -19.6500, 5.5900, 361.2600, 131.9500, -72.8300, -0.9800, -19.6500, 5.0600, 333.1500, 120.2400, -33.1400, -1.9900, -18.9900, 4.2900, 289.0700, 102.9200, 10.5800, -1.8600, -15.2300, 3.8000, 244.0100, 78.7100, 46.8600, -1.8600, -11.8000, 3.4200, 208.4600, 70.6000, 65.0300, -1.6500, -10.5500, 2.2300, 179.0700, 67.5600, 68.8800, -0.7000, -9.6300, 1.4700, 148.8900, 77.4900, 61.6800, 0.0800, -8.9400, 0.7200, 123.2800, 75.6000, 47.3500, 0.2700, -7.7900, 0.0500, 99.8600, 65.3000, 25.4000, 0.4300, -6.5400, -0.6100, 75.1700, 36.5200, 1.8000
};
int main(int argc, char **argv) {

    signal_t signal;            // Wrapper for raw input buffer
    ei_impulse_result_t result; // Used to store inference output
    EI_IMPULSE_ERROR res;       // Return code from inference

    // Calculate the length of the buffer
    size_t buf_len = sizeof(features) / sizeof(features[0]);

    // Make sure that the length of the buffer matches expected input length
    if (buf_len != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("ERROR: The size of the input buffer is not correct.\r\n");
        ei_printf("Expected %d items, but got %d\r\n",
                EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
                (int)buf_len);
        return 1;
    }

    run_classifier_init();

    // Assign callback function to fill buffer used for preprocessing/inference
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    // Perform DSP pre-processing and inference
    res = run_classifier(&signal, &result, false);

    // Print return code and how long it took to perform inference
    ei_printf("run_classifier returned: %d\r\n", res);
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_TRACKING_ENABLED == 1
    // Print the prediction results (object tracking)
    printf("Object tracking results:\n");
    for (uint32_t ix = 0; ix < result.postprocessed_output.object_tracking_output.open_traces_count; ix++) {
        ei_object_tracking_trace_t trace = result.postprocessed_output.object_tracking_output.open_traces[ix];
        printf("  %s (ID %d) [ x: %u, y: %u, width: %u, height: %u ]\n", trace.label, (int)trace.id, trace.x, trace.y, trace.width, trace.height);
    }

    if (result.postprocessed_output.object_tracking_output.open_traces_count == 0) {
        printf("    No objects found\n");
    }
#elif EI_CLASSIFIER_OBJECT_DETECTION == 1
    // Print the prediction results (object detection)
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
    ei_printf("Visual anomaly values: Mean : %.3f Max : %.3f\r\n",
    result.visual_ad_result.mean_value, result.visual_ad_result.max_value);
#endif

    return 0;
}

// Callback: fill a section of the out_ptr buffer when requested
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (features + offset)[i];
    }

    return EIDSP_OK;
}
