#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define MAX_N 30

void print_usage(const char *prog_name) {
    fprintf(stderr, "Usage: %s [-n integer] <floating_point_parameter>\n", prog_name);
}

// Function to calculate the standard deviation
double calculate_sigma(double *values, int count) {
    if (count == 0) return 0.0;

    double sum = 0.0, mean = 0.0, variance = 0.0;

    for (int i = 0; i < count; i++) {
        sum += values[i];
    }
    mean = sum / count;

    for (int i = 0; i < count; i++) {
        variance += (values[i] - mean) * (values[i] - mean);
    }
    variance /= count;

    return sqrt(variance);
}

// Function to calculate the accuracy
double calculate_accuracy(double *values, int count, double param) {
    if (count == 0) return 0.0;

    double sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += values[i];
    }
    double mean = sum / count;

    return fabs(mean - param);
}

int main(int argc, char *argv[]) {
    int n = 10; // Default value for -n option
    float param = 0.0;
    int opt;

    // Parse command-line options
    while ((opt = getopt(argc, argv, "n:")) != -1) {
        switch (opt) {
            case 'n':
                n = atoi(optarg);
                if (n > MAX_N) {
                    fprintf(stderr, "n cannot exceed %d. Setting n to %d.\n", MAX_N, MAX_N);
                    n = MAX_N;
                }
                break;
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    // Ensure a floating-point parameter is provided
    if (optind >= argc) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    param = atof(argv[optind]);

    // Process input from stdin and output to stdout
    char buffer[1024];
    double values[MAX_N];
    int count = 0, start = 0;

    while (fgets(buffer, sizeof(buffer), stdin)) {
        double value = atof(buffer);

        // Add the new value to the circular buffer
        values[(start + count) % n] = value;
        if (count < n) {
            count++;
        } else {
            start = (start + 1) % n;
        }

        // Only calculate and print after at least n samples have been read
        if (count >= n) {
            double sigma = calculate_sigma(values + start, n);
            double accuracy = calculate_accuracy(values + start, n, param);
            printf("Sigma: %.6f, Accuracy: %.6f\n", sigma, accuracy);
        }
    }

    return EXIT_SUCCESS;
}